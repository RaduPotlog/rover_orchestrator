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

#ifndef ROVER_MISSION_MANAGER_INFRASTRUCTURE_MISSION_REQUEST_HPP_
#define ROVER_MISSION_MANAGER_INFRASTRUCTURE_MISSION_REQUEST_HPP_

#include <optional>
#include <string>

#include <rover_msgs/srv/set_mission.hpp>

#include "rover_mission_manager/domain/mission.hpp"

namespace rover_mission_manager::infrastructure
{

/**
 * @brief Translate a SetMission request into a domain mission.
 *
 * The manager sends every goal in one frame (goal_frame_id) and has no TF buffer, so a
 * waypoint must either leave frame_id empty or name that frame; anything else is rejected
 * rather than silently driven to in the wrong frame. Leading slashes are ignored on both
 * sides, as tf2 does.
 *
 * @param error Set to the reason when std::nullopt is returned.
 */
std::optional<domain::Mission> missionFromRequest(
    const rover_msgs::srv::SetMission::Request & request, const std::string & goal_frame_id,
    const std::string & fallback_id, std::string & error);

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_MISSION_REQUEST_HPP_
