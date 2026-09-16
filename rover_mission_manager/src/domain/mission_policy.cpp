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

#include "rover_mission_manager/domain/mission_policy.hpp"

#include <algorithm>

namespace rover_mission_manager::domain
{

MissionPolicy::MissionPolicy(double abort_battery_fraction, bool require_lidar)
: abort_battery_fraction_(std::clamp(abort_battery_fraction, 0.0, 1.0)),
  require_lidar_(require_lidar)
{
}

MissionAction MissionPolicy::decide(const RoverConditions & conditions) const
{
    // A flat battery is checked first: holding would only keep discharging, and an engaged
    // motion lock must not mask the reason the mission is being abandoned.
    if (conditions.battery_fraction >= 0.0 &&
        conditions.battery_fraction < abort_battery_fraction_)
    {
        return MissionAction::kAbort;
    }

    // Checked before the lock so an engaged lock cannot mask a dead sensor in the status
    // message, and after the battery abort because an abort outranks any hold.
    if (conditions.lidar_health == SensorHealth::kUnhealthy ||
        (require_lidar_ && conditions.lidar_health == SensorHealth::kUnknown))
    {
        return MissionAction::kHold;
    }

    if (conditions.motion_locked) {
        return MissionAction::kHold;
    }

    return MissionAction::kProceed;
}

}  // namespace rover_mission_manager::domain
