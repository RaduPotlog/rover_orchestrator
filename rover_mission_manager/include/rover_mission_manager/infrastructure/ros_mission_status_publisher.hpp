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
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rover_msgs/msg/mission_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "rover_mission_manager/domain/ports/mission_status_publisher_port.hpp"

namespace rover_mission_manager::infrastructure
{

/**
 * @brief Publishes mission progress twice: a typed rover_msgs/MissionState on
 * `mission_state` for UIs (rover_drive_interface), and the original one-line std_msgs/String
 * on `mission_status` for logs and existing tools.
 *
 * Both are latched (transient_local, depth 1) so a tool attaching later still sees the
 * current state.
 *
 * Plain rclcpp publishers, not lifecycle ones, on purpose: the manager's on_deactivate and
 * on_shutdown cancel the mission, and that final CANCELLED state has to reach operators even
 * though the node is on its way out of ACTIVE.
 */
class RosMissionStatusPublisher : public domain::ports::MissionStatusPublisherPort
{
public:
    RosMissionStatusPublisher(
        rclcpp_lifecycle::LifecycleNode * node,
        const std::string & status_topic,
        const std::string & state_topic);

    void publish(const domain::Mission & mission) override;

private:
    rclcpp_lifecycle::LifecycleNode * node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<rover_msgs::msg::MissionState>::SharedPtr state_publisher_;
};

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_ROS_MISSION_STATUS_PUBLISHER_HPP_
