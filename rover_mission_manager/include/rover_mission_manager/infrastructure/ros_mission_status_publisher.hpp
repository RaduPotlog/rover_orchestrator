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

#ifndef ROVER_MISSION_MANAGER_INFRASTRUCTURE_ROS_MISSION_STATUS_PUBLISHER_HPP_
#define ROVER_MISSION_MANAGER_INFRASTRUCTURE_ROS_MISSION_STATUS_PUBLISHER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "rover_mission_manager/domain/ports/mission_status_publisher_port.hpp"

namespace rover_mission_manager::infrastructure
{

/**
 * @brief Publishes mission status as a std_msgs/String on `mission_status`.
 *
 * A String rather than a typed message because rover_msgs (in the rover_ros repository) has
 * no mission message and adding one is a cross-repo change. The port keeps that swap cheap.
 * Latched (transient_local) so an operator tool attaching later still sees the current state.
 */
class RosMissionStatusPublisher : public domain::ports::MissionStatusPublisherPort
{
public:
    RosMissionStatusPublisher(rclcpp::Node * node, const std::string & topic);

    void publish(const domain::Mission & mission) override;

private:
    rclcpp::Node * node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_ROS_MISSION_STATUS_PUBLISHER_HPP_
