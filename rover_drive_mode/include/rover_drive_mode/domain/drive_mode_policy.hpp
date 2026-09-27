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

#ifndef ROVER_DRIVE_MODE_DOMAIN_DRIVE_MODE_POLICY_HPP_
#define ROVER_DRIVE_MODE_DOMAIN_DRIVE_MODE_POLICY_HPP_

#include <optional>
#include <string>

#include "rover_drive_mode/domain/drive_mode.hpp"

namespace rover_drive_mode::domain
{

/** @brief What a mode needs from the rest of the system before it may be entered. */
struct Prerequisites
{
    /** @brief rover_mission_manager's set_mission service is on the graph. */
    bool mission_manager_available = false;

    /** @brief The teleop collision monitor is configured to run (use_teleop_guard). */
    bool teleop_guard_available = true;
};

/** @brief Outcome of a mode request. */
struct TransitionDecision
{
    bool accepted = false;
    std::string reason;
};

/**
 * @brief When the rover may enter a mode, and when it must leave one on its own.
 *
 * MANUAL is always reachable: it depends on nothing but the platform, and it is how the
 * operator gets the rover out of a spot where the lidar guard will not let it move.
 *
 * ASSISTED needs its collision monitor to be configured at all. Whether that monitor is
 * *healthy* is not a prerequisite on purpose: an unhealthy guard blocks motion, which is the
 * fail-safe outcome, and the operator sees it as GuardState::kNoData.
 *
 * AUTOMATIC needs the mission manager, since nothing else sends goals. When the manager
 * disappears for longer than the grace period while in AUTOMATIC, the mode falls back.
 */
class DriveModePolicy
{
public:
    explicit DriveModePolicy(
        bool require_mission_manager_for_automatic = true,
        double mission_manager_loss_grace_s = 2.0);

    TransitionDecision decide(DriveMode from, DriveMode to, const Prerequisites & prereqs) const;

    /**
     * @brief The mode an operator takeover or a forced fallback lands in: ASSISTED when its
     *        guard is available, MANUAL otherwise.
     */
    DriveMode fallbackMode(const Prerequisites & prereqs) const;

    /**
     * @brief A transition the rover must make without being asked.
     * @param mission_manager_missing_s How long the mission manager has been off the graph
     *        (0 when it is present).
     */
    std::optional<DriveMode> forcedTransition(
        DriveMode current, const Prerequisites & prereqs, double mission_manager_missing_s) const;

private:
    bool require_mission_manager_for_automatic_;
    double mission_manager_loss_grace_s_;
};

}  // namespace rover_drive_mode::domain

#endif  // ROVER_DRIVE_MODE_DOMAIN_DRIVE_MODE_POLICY_HPP_
