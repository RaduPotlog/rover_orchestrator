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

#ifndef ROVER_DRIVE_MODE_INFRASTRUCTURE_DRIVE_MODE_NODE_HPP_
#define ROVER_DRIVE_MODE_INFRASTRUCTURE_DRIVE_MODE_NODE_HPP_

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/drive_mode.hpp>
#include <rover_msgs/srv/set_drive_mode.hpp>
#include <rover_msgs/srv/set_mission.hpp>

#include "rover_drive_mode/application/drive_mode_use_case.hpp"
#include "rover_drive_mode/domain/guard_state.hpp"
#include "rover_drive_mode/drive_mode_parameters.hpp"

namespace rover_drive_mode::infrastructure
{

/**
 * @brief Owns the rover's driving mode (MANUAL / ASSISTED / AUTOMATIC) and routes commands.
 *
 * Sits in front of the two command inputs the platform already consumes, so the platform
 * needs no knowledge of modes:
 *
 *   web joystick -> [MANUAL]    -> teleop_driver_interface_cmd_vel_stamped
 *                -> [ASSISTED]  -> teleop collision monitor -> back here -> same output
 *                -> [AUTOMATIC] -> moving stick = takeover to ASSISTED, centred stick dropped
 *   Nav 2 collision monitor -> back here -> [AUTOMATIC only] -> nav_cmd_vel_stamped
 *
 * If this node dies, neither web teleop nor Nav 2 reaches the platform: twist_mux times both
 * inputs out. RC and the Foxglove joystick do not pass through here.
 */
class DriveModeNode : public rclcpp::Node
{
public:
    explicit DriveModeNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    using TwistStamped = geometry_msgs::msg::TwistStamped;
    using CollisionMonitorState = nav2_msgs::msg::CollisionMonitorState;

    void webTeleopCb(const TwistStamped::ConstSharedPtr & msg);
    void teleopGuardOutputCb(const TwistStamped::ConstSharedPtr & msg);
    void navigationGuardOutputCb(const TwistStamped::ConstSharedPtr & msg);

    void setDriveModeCb(
        const std::shared_ptr<rover_msgs::srv::SetDriveMode::Request> request,
        std::shared_ptr<rover_msgs::srv::SetDriveMode::Response> response);

    void timerCb();

    domain::Prerequisites prerequisites() const;

    /** @brief Refresh `running` from the graph; a monitor that went away forgets its state. */
    void refreshGuardReport(domain::GuardReport & report, const std::string & state_topic);

    void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

    std::shared_ptr<drive_mode::ParamListener> param_listener_;
    drive_mode::Params params_;

    std::unique_ptr<application::DriveModeUseCase> use_case_;

    rclcpp::Publisher<TwistStamped>::SharedPtr teleop_output_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr teleop_guard_input_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr navigation_output_pub_;
    rclcpp::Publisher<rover_msgs::msg::DriveMode>::SharedPtr drive_mode_pub_;

    rclcpp::Subscription<TwistStamped>::SharedPtr web_teleop_sub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr teleop_guard_output_sub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr navigation_guard_output_sub_;
    rclcpp::Subscription<CollisionMonitorState>::SharedPtr teleop_guard_state_sub_;
    rclcpp::Subscription<CollisionMonitorState>::SharedPtr navigation_guard_state_sub_;

    rclcpp::Service<rover_msgs::srv::SetDriveMode>::SharedPtr set_drive_mode_srv_;
    rclcpp::Client<rover_msgs::srv::SetMission>::SharedPtr mission_manager_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    diagnostic_updater::Updater diagnostic_updater_;

    domain::GuardReport teleop_guard_;
    domain::GuardReport navigation_guard_;

    std::size_t teleop_dropped_ = 0;
};

}  // namespace rover_drive_mode::infrastructure

#endif  // ROVER_DRIVE_MODE_INFRASTRUCTURE_DRIVE_MODE_NODE_HPP_
