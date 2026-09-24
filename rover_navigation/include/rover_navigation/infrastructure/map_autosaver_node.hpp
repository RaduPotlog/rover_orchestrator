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

#ifndef ROVER_NAVIGATION_INFRASTRUCTURE_MAP_AUTOSAVER_NODE_HPP_
#define ROVER_NAVIGATION_INFRASTRUCTURE_MAP_AUTOSAVER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rover_navigation/application/autosave_map_use_case.hpp"

namespace rover_navigation::infrastructure
{

/**
 * @brief Periodically persists the SLAM map by calling nav2_map_server's save_map service.
 *
 * Only useful while SLAM is running (`localization_source:=slam`); with a static map server
 * there is nothing to save. bringup.launch.py starts it only in that mode.
 *
 * A plain node rather than a lifecycle one: it holds no hardware and no in-flight action
 * that must be stopped on the way out. It only has a timer and a service client, and a save
 * interrupted by shutdown is simply repeated by the next run. Nothing needs to pause it
 * separately from slam_toolbox either, since bringup starts and stops both together.
 */
class MapAutosaverNode : public rclcpp::Node
{
public:
    explicit MapAutosaverNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /** @brief Declare parameters, build the use case and start the timer. */
    void initialize();

private:
    void autosaveCb();

    /** @brief `<namespace>/map`, matching what map_server publishes. */
    std::string resolveMapTopic() const;

    std::unique_ptr<application::AutosaveMapUseCase> autosave_map_use_case_;
    rclcpp::TimerBase::SharedPtr autosave_timer_;
};

}  // namespace rover_navigation::infrastructure

#endif  // ROVER_NAVIGATION_INFRASTRUCTURE_MAP_AUTOSAVER_NODE_HPP_
