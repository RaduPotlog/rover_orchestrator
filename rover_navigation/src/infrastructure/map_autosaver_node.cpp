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

#include "rover_navigation/infrastructure/map_autosaver_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rover_navigation/domain/map_autosave_policy.hpp"
#include "rover_navigation/infrastructure/nav2_map_saver_client.hpp"

namespace rover_navigation::infrastructure
{

namespace
{
constexpr std::chrono::duration<double> kSaveMapConnectionTimeout{1.0};
constexpr char kSaveMapService[] = "map_saver/save_map";
}  // namespace

MapAutosaverNode::MapAutosaverNode(
    const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
    this->declare_parameter<double>(
        "autosave_period", domain::MapAutosavePolicy::kMinPeriod.count());
    this->declare_parameter<std::string>("map_directory", "/maps/map");
}

std::string MapAutosaverNode::resolveMapTopic() const
{
    std::string ns = this->get_namespace();

    // get_namespace() is "/" at the root and "/rover" otherwise; both must end up as a
    // single-slash prefix so the topic matches what map_server publishes.
    if (ns.empty() || ns.back() != '/') {
        ns += "/";
    }

    return ns + "map";
}

void MapAutosaverNode::initialize()
{
    const auto requested_period =
        std::chrono::duration<double>(this->get_parameter("autosave_period").as_double());

    domain::MapAutosavePolicy policy(requested_period);

    if (policy.periodWasClamped()) {
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "autosave_period of " << requested_period.count()
                                  << " s is too short; using the minimum of "
                                  << policy.period().count() << " s.");
    }

    domain::MapSaveRequest request;
    request.map_topic = resolveMapTopic();
    request.map_url = this->get_parameter("map_directory").as_string();

    auto map_saver = std::make_shared<Nav2MapSaverClient>(
        this, kSaveMapService, kSaveMapConnectionTimeout);

    const auto period = policy.period();

    autosave_map_use_case_ = std::make_unique<application::AutosaveMapUseCase>(
        std::move(map_saver), policy, std::move(request));

    autosave_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MapAutosaverNode::autosaveCb, this));

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Map autosaver started: every " << period.count() << " s, topic '" << resolveMapTopic()
                                        << "' -> '"
                                        << this->get_parameter("map_directory").as_string()
                                        << "'.");
}

void MapAutosaverNode::autosaveCb()
{
    switch (autosave_map_use_case_->execute()) {
        case application::AutosaveOutcome::kRequested:
            RCLCPP_DEBUG(this->get_logger(), "Map save requested.");
            break;

        case application::AutosaveOutcome::kSkippedBackingOff:
            RCLCPP_DEBUG(this->get_logger(), "Skipping map save while backing off.");
            break;

        case application::AutosaveOutcome::kSkippedInFlight:
            RCLCPP_DEBUG(this->get_logger(), "Previous map save still pending; skipping.");
            break;

        case application::AutosaveOutcome::kSaveTimedOut:
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 10000,
                "'%s' never answered a save request (%d consecutive failures); backing off.",
                kSaveMapService, autosave_map_use_case_->policy().consecutiveFailures());
            break;

        case application::AutosaveOutcome::kSaverUnavailable:
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 10000,
                "'%s' unavailable (%d consecutive failures); backing off. Is SLAM running?",
                kSaveMapService, autosave_map_use_case_->policy().consecutiveFailures());
            break;
    }
}

}  // namespace rover_navigation::infrastructure
