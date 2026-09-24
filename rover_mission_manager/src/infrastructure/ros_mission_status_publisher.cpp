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

#include "rover_mission_manager/infrastructure/ros_mission_status_publisher.hpp"

#include <sstream>
#include <string>

namespace rover_mission_manager::infrastructure
{

namespace
{

uint8_t toMsgState(domain::MissionState state)
{
    using rover_msgs::msg::MissionState;
    switch (state) {
        case domain::MissionState::kIdle: return MissionState::IDLE;
        case domain::MissionState::kRunning: return MissionState::RUNNING;
        case domain::MissionState::kHeldByLock: return MissionState::HELD;
        case domain::MissionState::kSucceeded: return MissionState::SUCCEEDED;
        case domain::MissionState::kFailed: return MissionState::FAILED;
        case domain::MissionState::kCancelled: return MissionState::CANCELLED;
    }
    return MissionState::IDLE;
}

}  // namespace

RosMissionStatusPublisher::RosMissionStatusPublisher(
    rclcpp_lifecycle::LifecycleNode * node,
    const std::string & status_topic,
    const std::string & state_topic)
: node_(node)
{
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = rclcpp::create_publisher<std_msgs::msg::String>(*node_, status_topic, latched);
    state_publisher_ =
        rclcpp::create_publisher<rover_msgs::msg::MissionState>(*node_, state_topic, latched);
}

void RosMissionStatusPublisher::publish(const domain::Mission & mission)
{
    std::ostringstream out;
    out << "id=" << (mission.id().empty() ? "<none>" : mission.id())
        << " state=" << domain::toString(mission.state())
        << " waypoint=" << mission.currentIndex() << "/" << mission.waypoints().size();

    if (!mission.failureReason().empty()) {
        out << " reason=" << mission.failureReason();
    }

    std_msgs::msg::String msg;
    msg.data = out.str();

    publisher_->publish(msg);

    rover_msgs::msg::MissionState state;
    state.header.stamp = node_->now();
    state.mission_id = mission.id();
    state.state = toMsgState(mission.state());
    state.current_index = static_cast<uint32_t>(mission.currentIndex());
    state.total = static_cast<uint32_t>(mission.waypoints().size());
    state.message = mission.failureReason();
    state_publisher_->publish(state);

    RCLCPP_INFO_STREAM(node_->get_logger(), "Mission status: " << msg.data);
}

}  // namespace rover_mission_manager::infrastructure
