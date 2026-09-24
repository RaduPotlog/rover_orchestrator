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

#include <exception>
#include <memory>
#include <stdexcept>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_mission_manager/infrastructure/mission_manager_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;

    try {
        auto mission_manager_node =
            std::make_shared<rover_mission_manager::infrastructure::MissionManagerNode>(
                "mission_manager");

        // Autostart by default, so the node behaves as it did before it became a lifecycle
        // node: a manager that cannot configure (a bad tree, a missing plugin) exits non-zero
        // and the container restarts it, rather than idling unconfigured.
        if (mission_manager_node->autostart()) {
            using lifecycle_msgs::msg::State;
            if (mission_manager_node->configure().id() != State::PRIMARY_STATE_INACTIVE ||
                mission_manager_node->activate().id() != State::PRIMARY_STATE_ACTIVE)
            {
                throw std::runtime_error("the mission manager failed to start");
            }
        }

        // Single-threaded on purpose. Every callback here returns promptly -- the Nav 2 goal
        // is dispatched and polled without waiting -- and one thread keeps the timer, the
        // services and the action-client callbacks from ever running concurrently.
        rclcpp::spin(mission_manager_node->get_node_base_interface());
    } catch (const std::exception & e) {
        RCLCPP_FATAL_STREAM(
            rclcpp::get_logger("mission_manager"), "Caught exception: " << e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
