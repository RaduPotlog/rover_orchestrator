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

#include "rover_drive_mode/domain/drive_mode_policy.hpp"

#include <algorithm>

namespace rover_drive_mode::domain
{

DriveModePolicy::DriveModePolicy(
    bool require_mission_manager_for_automatic, double mission_manager_loss_grace_s)
: require_mission_manager_for_automatic_(require_mission_manager_for_automatic),
  mission_manager_loss_grace_s_(std::max(0.0, mission_manager_loss_grace_s))
{
}

TransitionDecision DriveModePolicy::decide(
    DriveMode from, DriveMode to, const Prerequisites & prereqs) const
{
    if (from == to) {
        return {true, std::string("Already in ") + toString(to) + "."};
    }

    switch (to) {
        case DriveMode::kManual:
            return {true, "Manual driving: no obstacle check on the web joystick."};

        case DriveMode::kAssisted:
            if (!prereqs.teleop_guard_available) {
                return {
                    false,
                    "Assisted driving is unavailable: the teleop collision monitor is disabled "
                    "(use_teleop_guard:=false)."};
            }
            return {true, "Assisted driving: the lidar slows and stops the web joystick."};

        case DriveMode::kAutomatic:
            if (require_mission_manager_for_automatic_ && !prereqs.mission_manager_available) {
                return {
                    false,
                    "Automatic driving needs the mission manager (set_mission), which is not "
                    "running. Is ROVER_ORCH_MISSION_MANAGER=true?"};
            }
            return {true, "Automatic driving: GoTo goals are accepted."};
    }

    return {false, "Unknown driving mode."};
}

DriveMode DriveModePolicy::fallbackMode(const Prerequisites & prereqs) const
{
    return prereqs.teleop_guard_available ? DriveMode::kAssisted : DriveMode::kManual;
}

std::optional<DriveMode> DriveModePolicy::forcedTransition(
    DriveMode current, const Prerequisites & prereqs, double mission_manager_missing_s) const
{
    if (current != DriveMode::kAutomatic || !require_mission_manager_for_automatic_) {
        return std::nullopt;
    }

    // The grace period rides out a respawn or a slow graph update; a missing manager means
    // nobody can cancel or finish the goal Nav 2 is driving, so the operator gets it back.
    if (!prereqs.mission_manager_available &&
        mission_manager_missing_s > mission_manager_loss_grace_s_)
    {
        return fallbackMode(prereqs);
    }

    return std::nullopt;
}

}  // namespace rover_drive_mode::domain
