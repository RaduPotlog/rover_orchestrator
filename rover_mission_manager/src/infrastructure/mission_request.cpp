// Copyright 2026 Mechatronics Academy
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

#include "rover_mission_manager/infrastructure/mission_request.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace rover_mission_manager::infrastructure
{

namespace
{

std::string stripLeadingSlash(const std::string & frame)
{
    const auto first = frame.find_first_not_of('/');
    return first == std::string::npos ? std::string() : frame.substr(first);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

std::optional<domain::Mission> missionFromRequest(
    const rover_msgs::srv::SetMission::Request & request, const std::string & goal_frame_id,
    const std::string & fallback_id, std::string & error)
{
    if (request.waypoints.empty()) {
        error = "A mission needs at least one waypoint.";
        return std::nullopt;
    }

    const auto goal_frame = stripLeadingSlash(goal_frame_id);
    std::vector<domain::Waypoint> waypoints;
    waypoints.reserve(request.waypoints.size());

    for (std::size_t i = 0; i < request.waypoints.size(); ++i) {
        const auto & pose = request.waypoints[i];
        const auto frame = stripLeadingSlash(pose.header.frame_id);

        if (!frame.empty() && frame != goal_frame) {
            error = "Waypoint " + std::to_string(i) + " is in frame '" + frame +
                    "', but goals are sent in '" + goal_frame + "'.";
            return std::nullopt;
        }

        const auto & p = pose.pose.position;
        const auto & q = pose.pose.orientation;
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) || norm < 1e-6) {
            error = "Waypoint " + std::to_string(i) + " has a non-finite position or a zero quaternion.";
            return std::nullopt;
        }

        waypoints.push_back(domain::Waypoint{p.x, p.y, yawFromQuaternion(q)});
    }

    auto id = request.mission_id.empty() ? fallback_id : request.mission_id;
    return domain::Mission(std::move(id), std::move(waypoints));
}

}  // namespace rover_mission_manager::infrastructure
