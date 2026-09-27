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

#ifndef ROVER_DRIVE_MODE_DOMAIN_GUARD_STATE_HPP_
#define ROVER_DRIVE_MODE_DOMAIN_GUARD_STATE_HPP_

#include "rover_drive_mode/domain/drive_mode.hpp"

namespace rover_drive_mode::domain
{

/** @brief The action a collision monitor last reported (nav2 CollisionMonitorState). */
enum class CollisionAction
{
    kDoNothing,
    kStop,
    kSlowdown,
    kLimit,
    kApproach,
};

/**
 * @brief What is known about one collision monitor.
 *
 * nav2_collision_monitor publishes its state only when the active zone changes, and only
 * while commands flow through it. So "no report yet" from a running monitor means nothing has
 * entered a zone (kClear), and the last report stays valid until the next one: a report's age
 * says nothing about the monitor's health. Whether it is running at all comes from the graph.
 */
struct GuardReport
{
    bool running = false;          ///< The monitor's state publisher is on the graph.
    bool received = false;         ///< At least one state report since it (re)appeared.
    CollisionAction action = CollisionAction::kDoNothing;
    bool source_invalid = false;   ///< The monitor stopped because its lidar data is stale.
};

/**
 * @brief The guard state to show for @p mode, from the monitor that guards it.
 *
 * MANUAL has no guard. ASSISTED is guarded by the teleop monitor and AUTOMATIC by Nav 2's.
 * A monitor that is not running, or that stopped on stale lidar data, is kNoData: motion is
 * blocked either way, and the operator needs to know it is not an obstacle.
 */
GuardState guardState(
    DriveMode mode, const GuardReport & teleop_guard, const GuardReport & navigation_guard);

}  // namespace rover_drive_mode::domain

#endif  // ROVER_DRIVE_MODE_DOMAIN_GUARD_STATE_HPP_
