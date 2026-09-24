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

#include <rclcpp/rclcpp.hpp>

#include "rover_mission_manager/infrastructure/mission_manager_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto mission_manager_node =
            std::make_shared<rover_mission_manager::infrastructure::MissionManagerNode>(
                "mission_manager");

        // initialize() needs shared_from_this() to seed the behavior tree blackboard, so it
        // cannot run inside the constructor.
        mission_manager_node->initialize();

        rclcpp::spin(mission_manager_node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL_STREAM(
            rclcpp::get_logger("mission_manager"), "Caught exception: " << e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
