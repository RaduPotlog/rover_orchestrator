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

#include "rover_navigation/infrastructure/map_autosaver_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto map_autosaver_node =
            std::make_shared<rover_navigation::infrastructure::MapAutosaverNode>("map_autosaver");
        map_autosaver_node->initialize();

        rclcpp::spin(map_autosaver_node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL_STREAM(
            rclcpp::get_logger("map_autosaver"), "Caught exception: " << e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
