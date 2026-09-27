// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rover_drive_mode/infrastructure/drive_mode_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "rover_drive_mode/infrastructure/ros_drive_mode_output.hpp"

namespace rover_drive_mode::infrastructure
{

namespace
{

// Same as rover_command_freshness_node's subscription and the UI's publisher: reliable, so a
// release burst of zeros is not lost; depth 10 so a burst survives a slow executor turn.
rclcpp::QoS commandQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
}

double steadySeconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

DriveModeNode::DriveModeNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options), diagnostic_updater_(this)
{
    param_listener_ =
        std::make_shared<drive_mode::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    const auto & topics = params_.topics;

    teleop_output_pub_ = this->create_publisher<TwistStamped>(topics.teleop_output, commandQos());
    teleop_guard_input_pub_ =
        this->create_publisher<TwistStamped>(topics.teleop_guard_input, commandQos());
    navigation_output_pub_ =
        this->create_publisher<TwistStamped>(topics.navigation_output, commandQos());

    // Latched: the UI and the mission manager join at any time and must learn the mode without
    // waiting for the next change.
    drive_mode_pub_ = this->create_publisher<rover_msgs::msg::DriveMode>(
        topics.drive_mode, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    auto output = std::make_shared<RosDriveModeOutput>(
        this, drive_mode_pub_, navigation_output_pub_);

    const auto initial_mode =
        domain::driveModeFromString(params_.default_mode).value_or(domain::DriveMode::kManual);

    use_case_ = std::make_unique<application::DriveModeUseCase>(
        std::move(output),
        domain::DriveModePolicy(
            params_.require_mission_manager_for_automatic, params_.mission_manager_loss_grace),
        domain::TakeoverThreshold{
            params_.takeover_threshold.linear, params_.takeover_threshold.angular},
        initial_mode, "boot default");

    mission_manager_client_ =
        this->create_client<rover_msgs::srv::SetMission>(params_.mission_manager_service);

    web_teleop_sub_ = this->create_subscription<TwistStamped>(
        topics.web_teleop, commandQos(),
        std::bind(&DriveModeNode::webTeleopCb, this, std::placeholders::_1));

    teleop_guard_output_sub_ = this->create_subscription<TwistStamped>(
        topics.teleop_guard_output, commandQos(),
        std::bind(&DriveModeNode::teleopGuardOutputCb, this, std::placeholders::_1));

    navigation_guard_output_sub_ = this->create_subscription<TwistStamped>(
        topics.navigation_guard_output, commandQos(),
        std::bind(&DriveModeNode::navigationGuardOutputCb, this, std::placeholders::_1));

    // nav2_collision_monitor publishes its state only on a zone change, so none may be lost:
    // reliable, matching its reliable publisher.
    teleop_guard_state_sub_ = this->create_subscription<CollisionMonitorState>(
        topics.teleop_guard_state, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const CollisionMonitorState::ConstSharedPtr & msg) {
            applyCollisionMonitorState(*msg, teleop_guard_);
            use_case_->onGuardReports(teleop_guard_, navigation_guard_);
        });

    navigation_guard_state_sub_ = this->create_subscription<CollisionMonitorState>(
        topics.navigation_guard_state, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const CollisionMonitorState::ConstSharedPtr & msg) {
            applyCollisionMonitorState(*msg, navigation_guard_);
            use_case_->onGuardReports(teleop_guard_, navigation_guard_);
        });

    set_drive_mode_srv_ = this->create_service<rover_msgs::srv::SetDriveMode>(
        "set_drive_mode", std::bind(
                              &DriveModeNode::setDriveModeCb, this, std::placeholders::_1,
                              std::placeholders::_2));

    refreshGuardReport(teleop_guard_, topics.teleop_guard_state);
    refreshGuardReport(navigation_guard_, topics.navigation_guard_state);
    use_case_->onGuardReports(teleop_guard_, navigation_guard_);
    use_case_->start(prerequisites());

    const auto period = std::chrono::duration<double>(1.0 / params_.timer_frequency);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DriveModeNode::timerCb, this));

    diagnostic_updater_.setHardwareID("Drive Mode");
    diagnostic_updater_.add("Drive mode", this, &DriveModeNode::diagnose);

    RCLCPP_INFO(
        this->get_logger(), "Drive mode manager ready in %s (%s); web joystick on '%s'.",
        domain::toString(use_case_->mode()), use_case_->reason().c_str(),
        web_teleop_sub_->get_topic_name());
}

domain::Prerequisites DriveModeNode::prerequisites() const
{
    domain::Prerequisites prereqs;
    prereqs.mission_manager_available = mission_manager_client_->service_is_ready();
    prereqs.teleop_guard_available = params_.use_teleop_guard;
    return prereqs;
}

void DriveModeNode::refreshGuardReport(domain::GuardReport & report, const std::string & topic)
{
    const bool running = this->count_publishers(topic) > 0;

    // A restarted monitor starts again from "nothing in any zone"; a report from its previous
    // life (say STOP) would otherwise stick until the next zone change.
    if (!running) {
        report = domain::GuardReport{};
    }

    report.running = running;
}

void DriveModeNode::webTeleopCb(const TwistStamped::ConstSharedPtr & msg)
{
    const auto mode_before = use_case_->mode();
    const auto route = use_case_->onTeleop(
        domain::Velocity{msg->twist.linear.x, msg->twist.angular.z}, prerequisites());

    if (use_case_->mode() != mode_before) {
        RCLCPP_WARN(
            this->get_logger(), "Operator takeover: %s -> %s.", domain::toString(mode_before),
            domain::toString(use_case_->mode()));
    }

    switch (route) {
        case domain::TeleopRoute::kDirect: teleop_output_pub_->publish(*msg); break;
        case domain::TeleopRoute::kGuard: teleop_guard_input_pub_->publish(*msg); break;
        case domain::TeleopRoute::kDrop: ++teleop_dropped_; break;
    }
}

void DriveModeNode::teleopGuardOutputCb(const TwistStamped::ConstSharedPtr & msg)
{
    // The monitor passes the operator's header through, so the platform's freshness check still
    // measures the whole browser-to-platform latency, guard included.
    if (use_case_->acceptGuarded(domain::GuardedSource::kTeleop)) {
        teleop_output_pub_->publish(*msg);
    }
}

void DriveModeNode::navigationGuardOutputCb(const TwistStamped::ConstSharedPtr & msg)
{
    if (use_case_->acceptGuarded(domain::GuardedSource::kNavigation)) {
        navigation_output_pub_->publish(*msg);
    }
}

void DriveModeNode::setDriveModeCb(
    const std::shared_ptr<rover_msgs::srv::SetDriveMode::Request> request,
    std::shared_ptr<rover_msgs::srv::SetDriveMode::Response> response)
{
    const auto requested = fromMsgMode(request->mode);

    if (!requested.has_value()) {
        response->success = false;
        response->message =
            "Unknown mode " + std::to_string(request->mode) +
            "; use rover_msgs/DriveMode MANUAL (1), ASSISTED (2) or AUTOMATIC (3).";
    } else {
        const auto mode_before = use_case_->mode();

        // request() publishes the new mode before this returns, so a caller that sees success
        // can rely on drive_mode already carrying it.
        const auto decision = use_case_->request(*requested, prerequisites());
        response->success = decision.accepted;
        response->message = decision.reason;

        if (!decision.accepted) {
            RCLCPP_WARN(
                this->get_logger(), "Refused %s: %s", domain::toString(*requested),
                decision.reason.c_str());
        } else if (use_case_->mode() != mode_before) {
            RCLCPP_INFO(
                this->get_logger(), "Drive mode %s -> %s (operator request).",
                domain::toString(mode_before), domain::toString(use_case_->mode()));
        }
    }

    response->mode = toMsgMode(use_case_->mode());
}

void DriveModeNode::timerCb()
{
    refreshGuardReport(teleop_guard_, params_.topics.teleop_guard_state);
    refreshGuardReport(navigation_guard_, params_.topics.navigation_guard_state);
    use_case_->onGuardReports(teleop_guard_, navigation_guard_);

    const auto mode_before = use_case_->mode();
    use_case_->tick(prerequisites(), steadySeconds());

    if (use_case_->mode() != mode_before) {
        RCLCPP_WARN(
            this->get_logger(), "Drive mode %s -> %s (%s).", domain::toString(mode_before),
            domain::toString(use_case_->mode()), use_case_->reason().c_str());
    }
}

void DriveModeNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    using diagnostic_msgs::msg::DiagnosticStatus;

    const auto mode = use_case_->mode();
    const auto guard = use_case_->guard();
    const auto prereqs = prerequisites();

    status.add("Mode", domain::toString(mode));
    status.add("Reason", use_case_->reason());
    status.add("Guard", domain::toString(guard));
    status.add("Teleop guard running", teleop_guard_.running);
    status.add("Navigation guard running", navigation_guard_.running);
    status.add("Mission manager available", prereqs.mission_manager_available);
    status.add("Centred-stick commands dropped in AUTOMATIC", teleop_dropped_);

    if (guard == domain::GuardState::kNoData) {
        status.summary(
            DiagnosticStatus::WARN,
            std::string(domain::toString(mode)) +
                ": collision monitor not running or no lidar data; motion is blocked.");
        return;
    }

    status.summary(
        DiagnosticStatus::OK,
        std::string(domain::toString(mode)) + ", guard " + domain::toString(guard) + ".");
}

}  // namespace rover_drive_mode::infrastructure
