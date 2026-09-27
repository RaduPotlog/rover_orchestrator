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

#include "rover_drive_mode/infrastructure/ros_drive_mode_output.hpp"

#include <utility>

namespace rover_drive_mode::infrastructure
{

using rover_msgs::msg::DriveMode;

std::uint8_t toMsgMode(domain::DriveMode mode)
{
    switch (mode) {
        case domain::DriveMode::kManual: return DriveMode::MANUAL;
        case domain::DriveMode::kAssisted: return DriveMode::ASSISTED;
        case domain::DriveMode::kAutomatic: return DriveMode::AUTOMATIC;
    }

    return 0;
}

std::optional<domain::DriveMode> fromMsgMode(std::uint8_t mode)
{
    switch (mode) {
        case DriveMode::MANUAL: return domain::DriveMode::kManual;
        case DriveMode::ASSISTED: return domain::DriveMode::kAssisted;
        case DriveMode::AUTOMATIC: return domain::DriveMode::kAutomatic;
        default: return std::nullopt;
    }
}

std::uint8_t toMsgGuard(domain::GuardState guard)
{
    switch (guard) {
        case domain::GuardState::kBypassed: return DriveMode::GUARD_BYPASSED;
        case domain::GuardState::kClear: return DriveMode::GUARD_CLEAR;
        case domain::GuardState::kSlowing: return DriveMode::GUARD_SLOWING;
        case domain::GuardState::kStopped: return DriveMode::GUARD_STOPPED;
        case domain::GuardState::kNoData: return DriveMode::GUARD_NO_DATA;
    }

    return DriveMode::GUARD_NO_DATA;
}

void applyCollisionMonitorState(
    const nav2_msgs::msg::CollisionMonitorState & msg, domain::GuardReport & report)
{
    using nav2_msgs::msg::CollisionMonitorState;

    report.received = true;

    switch (msg.action_type) {
        case CollisionMonitorState::STOP: report.action = domain::CollisionAction::kStop; break;
        case CollisionMonitorState::SLOWDOWN:
            report.action = domain::CollisionAction::kSlowdown;
            break;
        case CollisionMonitorState::APPROACH:
            report.action = domain::CollisionAction::kApproach;
            break;
        case CollisionMonitorState::LIMIT: report.action = domain::CollisionAction::kLimit; break;
        case CollisionMonitorState::DO_NOTHING:
        default: report.action = domain::CollisionAction::kDoNothing; break;
    }

    report.source_invalid = msg.polygon_name == "invalid source";
}

RosDriveModeOutput::RosDriveModeOutput(
    rclcpp::Node * node,
    rclcpp::Publisher<rover_msgs::msg::DriveMode>::SharedPtr drive_mode_pub,
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr navigation_output_pub)
: node_(node),
  drive_mode_pub_(std::move(drive_mode_pub)),
  navigation_output_pub_(std::move(navigation_output_pub))
{
}

void RosDriveModeOutput::publishStatus(const domain::ports::DriveModeStatus & status)
{
    DriveMode msg;
    msg.header.stamp = node_->now();
    msg.mode = toMsgMode(status.mode);
    msg.guard = toMsgGuard(status.guard);
    msg.reason = status.reason;
    drive_mode_pub_->publish(msg);
}

void RosDriveModeOutput::stopNavigation()
{
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = node_->now();
    navigation_output_pub_->publish(zero);
}

}  // namespace rover_drive_mode::infrastructure
