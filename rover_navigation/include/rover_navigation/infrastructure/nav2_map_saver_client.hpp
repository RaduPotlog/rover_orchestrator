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

#ifndef ROVER_NAVIGATION_INFRASTRUCTURE_NAV2_MAP_SAVER_CLIENT_HPP_
#define ROVER_NAVIGATION_INFRASTRUCTURE_NAV2_MAP_SAVER_CLIENT_HPP_

#include <chrono>
#include <string>

#include <nav2_msgs/srv/save_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_navigation/domain/ports/map_saver_port.hpp"

namespace rover_navigation::infrastructure
{

/** @brief MapSaverPort backed by nav2_map_server's `map_saver/save_map` service. */
class Nav2MapSaverClient : public domain::ports::MapSaverPort
{
public:
    using SaveMapSrv = nav2_msgs::srv::SaveMap;

    Nav2MapSaverClient(
        rclcpp::Node * node,
        const std::string & service_name,
        std::chrono::duration<double> connection_timeout);

    /**
     * @brief Dispatch a save request.
     *
     * The request is sent asynchronously and the reply is discarded: map_saver writes the
     * file itself, and blocking the timer thread on the response would stall the node. A
     * false return therefore means "could not reach the saver", not "the write failed".
     */
    bool save(const domain::MapSaveRequest & request) override;

private:
    rclcpp::Node * node_;
    rclcpp::Client<SaveMapSrv>::SharedPtr client_;
    std::chrono::duration<double> connection_timeout_;
};

}  // namespace rover_navigation::infrastructure

#endif  // ROVER_NAVIGATION_INFRASTRUCTURE_NAV2_MAP_SAVER_CLIENT_HPP_
