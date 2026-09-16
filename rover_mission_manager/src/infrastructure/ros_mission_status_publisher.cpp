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

RosMissionStatusPublisher::RosMissionStatusPublisher(
    rclcpp::Node * node, const std::string & topic)
: node_(node)
{
    publisher_ = node_->create_publisher<std_msgs::msg::String>(
        topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
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

    RCLCPP_INFO_STREAM(node_->get_logger(), "Mission status: " << msg.data);
}

}  // namespace rover_mission_manager::infrastructure
