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

#ifndef ROVER_DRIVE_MODE_DOMAIN_COMMAND_ROUTER_HPP_
#define ROVER_DRIVE_MODE_DOMAIN_COMMAND_ROUTER_HPP_

#include <optional>

#include "rover_drive_mode/domain/drive_mode.hpp"

namespace rover_drive_mode::domain
{

/** @brief A planar velocity command (m/s, rad/s). */
struct Velocity
{
    double linear = 0.0;
    double angular = 0.0;
};

/** @brief Below these magnitudes a web joystick command counts as "stick centred". */
struct TakeoverThreshold
{
    double linear = 0.02;
    double angular = 0.02;
};

/** @brief Where a web joystick command goes. */
enum class TeleopRoute
{
    kDirect,  ///< Straight to the platform's teleop input (MANUAL).
    kGuard,   ///< Into the teleop collision monitor (ASSISTED).
    kDrop,    ///< Nowhere (a centred stick in AUTOMATIC).
};

struct TeleopDecision
{
    TeleopRoute route = TeleopRoute::kDrop;

    /** @brief Set when the command is an operator takeover: the mode to switch to first. */
    std::optional<DriveMode> switch_to;
};

/** @brief Which collision monitor a guarded command came back from. */
enum class GuardedSource
{
    kTeleop,
    kNavigation,
};

/** @brief True when the command is more than a centred stick. */
bool isMotion(const Velocity & cmd, const TakeoverThreshold & threshold);

/**
 * @brief Route a web joystick command for the current mode.
 *
 * In AUTOMATIC a moving stick is a takeover: the command is routed as if already in
 * @p takeover_mode, and the caller switches to it. A centred stick is dropped so the release
 * burst the UI sends does not take over by itself.
 */
TeleopDecision routeTeleop(
    DriveMode mode, const Velocity & cmd, const TakeoverThreshold & threshold,
    DriveMode takeover_mode);

/**
 * @brief Whether a command coming back out of a collision monitor may reach the platform.
 *
 * The last hop is gated as well as the first, so a monitor still flushing commands (or its
 * stop_pub_timeout zeros) after a mode change cannot drive the rover in the new mode.
 */
bool forwardGuarded(GuardedSource source, DriveMode mode);

}  // namespace rover_drive_mode::domain

#endif  // ROVER_DRIVE_MODE_DOMAIN_COMMAND_ROUTER_HPP_
