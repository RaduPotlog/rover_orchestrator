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

#include "rover_navigation/infrastructure/nav2_map_saver_client.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace rover_navigation::infrastructure
{

Nav2MapSaverClient::Nav2MapSaverClient(
    rclcpp::Node * node,
    const std::string & service_name,
    std::chrono::duration<double> connection_timeout)
: node_(node), connection_timeout_(connection_timeout)
{
    client_ = node_->create_client<SaveMapSrv>(service_name);
}

bool Nav2MapSaverClient::save(const domain::MapSaveRequest & request)
{
    const auto timeout =
        std::chrono::duration_cast<std::chrono::nanoseconds>(connection_timeout_);

    if (!client_->wait_for_service(timeout)) {
        return false;
    }

    auto srv_request = std::make_shared<SaveMapSrv::Request>();
    srv_request->map_topic = request.map_topic;
    srv_request->map_url = request.map_url;
    srv_request->image_format = request.image_format;
    srv_request->map_mode = request.map_mode;
    srv_request->free_thresh = request.free_thresh;
    srv_request->occupied_thresh = request.occupied_thresh;

    client_->async_send_request(srv_request);
    return true;
}

}  // namespace rover_navigation::infrastructure
