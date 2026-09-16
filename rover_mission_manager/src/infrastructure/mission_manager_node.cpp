// Copyright 2025 Mechatronics Academy
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

#include "rover_mission_manager/infrastructure/mission_manager_node.hpp"

#include <any>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/utils/shared_library.h>

#include "rover_mission_manager/domain/mission.hpp"
#include "rover_mission_manager/domain/mission_policy.hpp"
#include "rover_mission_manager/infrastructure/nav2_navigation_adapter.hpp"
#include "rover_mission_manager/infrastructure/ros_mission_status_publisher.hpp"

namespace rover_mission_manager::infrastructure
{

MissionManagerNode::MissionManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options),
  motion_locked_(true),
  motion_lock_received_(false),
  motion_lock_stamp_ns_(0),
  battery_fraction_(-1.0),
  lidar_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  lidar_status_received_(false),
  lidar_stamp_ns_(0)
{
    param_listener_ =
        std::make_shared<mission_manager::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
}

std::string MissionManagerNode::resolveGoalFrameId() const
{
    if (!params_.goal_frame_id.empty()) {
        return params_.goal_frame_id;
    }

    std::string ns = this->get_namespace();

    // get_namespace() is "/" at the root and "/rover" otherwise. Strip the leading slash:
    // Nav 2 frame ids are "rover/odom", not "/rover/odom".
    if (!ns.empty() && ns.front() == '/') {
        ns.erase(0, 1);
    }

    return ns.empty() ? "odom" : ns + "/odom";
}

void MissionManagerNode::initialize()
{
    RCLCPP_INFO(this->get_logger(), "Initializing mission manager.");

    registerBehaviorTree();

    const std::map<std::string, std::any> mission_blackboard = {
        {"MOTION_LOCK_TOPIC", params_.motion_lock_topic},
        {"MOTION_LOCK_TIMEOUT", params_.motion_lock_timeout},
    };

    mission_tree_runner_ = std::make_unique<BehaviorTreeRunner>(
        params_.tree_name, mission_blackboard, static_cast<unsigned>(params_.bt_server_port));
    // BT leaf plugins (rover_navigation's IsMotionLocked, and any nav2_behavior_tree plugin
    // listed in ros_plugin_libs) look the node handle up on the blackboard under "node".
    mission_tree_runner_->initialize(factory_, [this](BT::Blackboard::Ptr blackboard) {
        blackboard->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        blackboard->set<std::chrono::milliseconds>(
            "server_timeout",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(params_.ros_communication_timeout.response)));
        blackboard->set<std::chrono::milliseconds>(
            "wait_for_service_timeout",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(params_.ros_communication_timeout.availability)));
    });

    const auto goal_frame_id = resolveGoalFrameId();

    auto navigation = std::make_shared<Nav2NavigationAdapter>(
        this, params_.navigate_to_pose_action, goal_frame_id,
        std::chrono::duration<double>(params_.ros_communication_timeout.availability));

    auto status_publisher =
        std::make_shared<RosMissionStatusPublisher>(this, params_.mission_status_topic);

    run_mission_use_case_ = std::make_unique<application::RunMissionUseCase>(
        std::move(navigation), std::move(status_publisher),
        domain::MissionPolicy(params_.abort_battery_fraction, params_.require_lidar));

    motion_lock_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        params_.motion_lock_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&MissionManagerNode::motionLockCb, this, std::placeholders::_1));

    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        params_.battery_topic, rclcpp::SensorDataQoS(),
        std::bind(&MissionManagerNode::batteryCb, this, std::placeholders::_1));

    // Depth 20, not KeepLast(1): /diagnostics is shared by every node on the rover, so a
    // depth-1 queue would routinely drop the one status this node watches.
    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        params_.lidar_health_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        std::bind(&MissionManagerNode::diagnosticsCb, this, std::placeholders::_1));

    run_mission_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "run_mission", std::bind(
                           &MissionManagerNode::runMissionCb, this, std::placeholders::_1,
                           std::placeholders::_2));

    const auto timer_period = std::chrono::duration<double>(1.0 / params_.timer_frequency);
    mission_tree_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&MissionManagerNode::timerCb, this));

    RCLCPP_INFO_STREAM(
        this->get_logger(), "Mission manager ready: ticking '"
                                << params_.tree_name << "' at " << params_.timer_frequency
                                << " Hz, goals in frame '" << goal_frame_id << "', Groot2 on "
                                << mission_tree_runner_->grootPort() << ".");
}

void MissionManagerNode::registerBehaviorTree()
{
    if (params_.bt_project_path.empty()) {
        throw std::runtime_error("'bt_project_path' is not set; nothing to tick.");
    }

    // Plain BT.CPP plugins first: they need nothing from ROS.
    for (const auto & plugin : params_.plugin_libs) {
        factory_.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }

    // nav2_behavior_tree's plugins read the node handle and their timeouts off the
    // blackboard of the tree they are built into, so those entries are seeded in
    // BehaviorTreeRunner; here we only need the libraries loaded.
    for (const auto & plugin : params_.ros_plugin_libs) {
        factory_.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }

    factory_.registerBehaviorTreeFromFile(params_.bt_project_path);

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "BehaviorTree registered from '" << params_.bt_project_path << "'.");
}

void MissionManagerNode::motionLockCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    motion_locked_ = msg->data;
    motion_lock_stamp_ns_ = this->now().nanoseconds();
    motion_lock_received_ = true;
}

void MissionManagerNode::batteryCb(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    // BatteryState::percentage is 0..1 in REP-0147; NaN means "not measured".
    if (!std::isnan(msg->percentage)) {
        battery_fraction_ = static_cast<double>(msg->percentage);
    }
}

void MissionManagerNode::diagnosticsCb(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    for (const auto & status : msg->status) {
        if (status.name != params_.lidar_status_name) {
            continue;
        }

        lidar_level_ = static_cast<unsigned char>(status.level);
        lidar_stamp_ns_ = this->now().nanoseconds();
        lidar_status_received_ = true;
        return;
    }
}

domain::RoverConditions MissionManagerNode::currentConditions() const
{
    domain::RoverConditions conditions;

    // Fail-safe: no message yet, or a stale one, both count as locked -- the same rule
    // rover_twist_mux applies, so the manager never believes it may drive when the mux does
    // not.
    if (!motion_lock_received_) {
        conditions.motion_locked = true;
    } else {
        const auto age_ns = this->now().nanoseconds() - motion_lock_stamp_ns_.load();
        const auto timeout_ns =
            static_cast<rcl_time_point_value_t>(params_.motion_lock_timeout * 1e9);

        conditions.motion_locked = (age_ns > timeout_ns) || motion_locked_.load();
    }

    conditions.battery_fraction = battery_fraction_.load();

    // Deliberately NOT fail-safe on "never seen", unlike the motion lock: the rover runs
    // without a lidar whenever ROVER_USE_LIDAR is false, and rover_lidar sits behind a 10 s
    // TimerAction in rover_bringup even when it is true. require_lidar is the opt-in.
    if (!lidar_status_received_) {
        conditions.lidar_health = domain::SensorHealth::kUnknown;
    } else {
        const auto age_ns = this->now().nanoseconds() - lidar_stamp_ns_.load();
        const auto timeout_ns =
            static_cast<rcl_time_point_value_t>(params_.lidar_health_timeout * 1e9);
        const auto level = lidar_level_.load();

        // OK and WARN are both usable: rover_lidar raises WARN for a sparse cloud or a rate
        // below min_rate_ratio, which degrades the costmaps but does not invalidate them.
        const bool usable = level == diagnostic_msgs::msg::DiagnosticStatus::OK ||
                            level == diagnostic_msgs::msg::DiagnosticStatus::WARN;

        conditions.lidar_health = (age_ns > timeout_ns || !usable)
                                      ? domain::SensorHealth::kUnhealthy
                                      : domain::SensorHealth::kHealthy;
    }

    return conditions;
}

void MissionManagerNode::runMissionCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!request->data) {
        run_mission_use_case_->cancel();
        response->success = true;
        response->message = "Mission cancelled.";
        return;
    }

    // Placeholder mission source. rover_msgs has no mission message yet (see README), so
    // until one exists this service only starts the demo mission on the blackboard.
    response->success = false;
    response->message =
        "No mission source configured. Publish waypoints through a mission interface, or "
        "drive the tree directly; see rover_mission_manager/README.md.";

    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
}

void MissionManagerNode::timerCb()
{
    run_mission_use_case_->tick(currentConditions());

    mission_tree_runner_->tickOnce();

    if (mission_tree_runner_->treeStatus() == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Mission behavior tree returned FAILURE.");
    }
}

}  // namespace rover_mission_manager::infrastructure
