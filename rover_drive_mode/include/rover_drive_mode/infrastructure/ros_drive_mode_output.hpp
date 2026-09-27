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

#ifndef ROVER_DRIVE_MODE_INFRASTRUCTURE_ROS_DRIVE_MODE_OUTPUT_HPP_
#define ROVER_DRIVE_MODE_INFRASTRUCTURE_ROS_DRIVE_MODE_OUTPUT_HPP_

#include <cstdint>
#include <optional>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/drive_mode.hpp>

#include "rover_drive_mode/domain/drive_mode.hpp"
#include "rover_drive_mode/domain/guard_state.hpp"
#include "rover_drive_mode/domain/ports/drive_mode_output_port.hpp"

namespace rover_drive_mode::infrastructure
{

/** @brief rover_msgs/DriveMode mode constant for a domain mode. */
std::uint8_t toMsgMode(domain::DriveMode mode);

/** @brief Domain mode for a rover_msgs/DriveMode constant; nullopt for anything else. */
std::optional<domain::DriveMode> fromMsgMode(std::uint8_t mode);

std::uint8_t toMsgGuard(domain::GuardState guard);

/**
 * @brief Fold a nav2_msgs/CollisionMonitorState into a report.
 *
 * "invalid source" is the polygon name nav2_collision_monitor stops under when an observation
 * source times out; it is a data problem, not an obstacle.
 */
void applyCollisionMonitorState(
    const nav2_msgs::msg::CollisionMonitorState & msg, domain::GuardReport & report);

/** @brief Publishes the use case's side effects on ROS topics. */
class RosDriveModeOutput : public domain::ports::DriveModeOutputPort
{
public:
    RosDriveModeOutput(
        rclcpp::Node * node,
        rclcpp::Publisher<rover_msgs::msg::DriveMode>::SharedPtr drive_mode_pub,
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr navigation_output_pub);

    void publishStatus(const domain::ports::DriveModeStatus & status) override;

    void stopNavigation() override;

private:
    rclcpp::Node * node_;
    rclcpp::Publisher<rover_msgs::msg::DriveMode>::SharedPtr drive_mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr navigation_output_pub_;
};

}  // namespace rover_drive_mode::infrastructure

#endif  // ROVER_DRIVE_MODE_INFRASTRUCTURE_ROS_DRIVE_MODE_OUTPUT_HPP_
