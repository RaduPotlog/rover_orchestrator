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

#ifndef ROVER_DRIVE_MODE_DOMAIN_DRIVE_MODE_HPP_
#define ROVER_DRIVE_MODE_DOMAIN_DRIVE_MODE_HPP_

#include <optional>
#include <string>

namespace rover_drive_mode::domain
{

/** @brief Who decides where the rover goes, and whether the lidar may veto it. */
enum class DriveMode
{
    kManual,     ///< Web joystick straight to the platform, no obstacle check.
    kAssisted,   ///< Web joystick through the teleop collision monitor (slow down, stop).
    kAutomatic,  ///< Nav 2 drives (GoTo), through its own collision monitor.
};

/** @brief What the collision monitor guarding the active mode is doing. */
enum class GuardState
{
    kBypassed,  ///< MANUAL: no collision monitor in the path.
    kClear,     ///< Nothing in the slow-down or stop zones.
    kSlowing,   ///< Obstacle in the slow-down zone; speed is scaled down.
    kStopped,   ///< Obstacle in the stop zone; motion is blocked.
    kNoData,    ///< Monitor not running, or no lidar data: motion is blocked.
};

const char * toString(DriveMode mode);

const char * toString(GuardState guard);

/** @brief "manual", "assisted" or "automatic" (as in the parameters); nullopt otherwise. */
std::optional<DriveMode> driveModeFromString(const std::string & name);

}  // namespace rover_drive_mode::domain

#endif  // ROVER_DRIVE_MODE_DOMAIN_DRIVE_MODE_HPP_
