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
#include <utility>

namespace rover_navigation::infrastructure
{

namespace
{
// Longer than any save takes; a request older than this has no reply coming.
constexpr std::chrono::minutes kPendingRequestLifetime{5};
}  // namespace

Nav2MapSaverClient::Nav2MapSaverClient(
    rclcpp::Node * node,
    const std::string & service_name,
    std::chrono::duration<double> connection_timeout)
: node_(node), connection_timeout_(connection_timeout)
{
    client_ = node_->create_client<SaveMapSrv>(service_name);
}

bool Nav2MapSaverClient::save(const domain::MapSaveRequest & request, SaveDoneCallback on_done)
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

    // A saver that never answers would otherwise leave its requests pending forever.
    client_->prune_requests_older_than(
        std::chrono::system_clock::now() - kPendingRequestLifetime);

    client_->async_send_request(
        srv_request,
        [this, map_url = request.map_url, on_done = std::move(on_done)](
            rclcpp::Client<SaveMapSrv>::SharedFuture future) {
            responseCb(map_url, on_done, future);
        });
    return true;
}

void Nav2MapSaverClient::responseCb(
    const std::string & map_url, const SaveDoneCallback & on_done,
    rclcpp::Client<SaveMapSrv>::SharedFuture future)
{
    const bool written = future.get()->result;

    if (!written) {
        RCLCPP_WARN_STREAM_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 10000,
            "map_saver failed to write the map to '"
                << map_url << "'; backing off. Check that its directory exists and is writable.");
    }

    if (on_done) {
        on_done(written);
    }
}

}  // namespace rover_navigation::infrastructure
