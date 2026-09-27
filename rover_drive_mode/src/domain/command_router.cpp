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

#include "rover_drive_mode/domain/command_router.hpp"

#include <cmath>

namespace rover_drive_mode::domain
{

namespace
{

TeleopRoute routeFor(DriveMode mode)
{
    return mode == DriveMode::kManual ? TeleopRoute::kDirect : TeleopRoute::kGuard;
}

}  // namespace

bool isMotion(const Velocity & cmd, const TakeoverThreshold & threshold)
{
    return std::abs(cmd.linear) > threshold.linear || std::abs(cmd.angular) > threshold.angular;
}

TeleopDecision routeTeleop(
    DriveMode mode, const Velocity & cmd, const TakeoverThreshold & threshold,
    DriveMode takeover_mode)
{
    TeleopDecision decision;

    switch (mode) {
        case DriveMode::kManual:
        case DriveMode::kAssisted:
            decision.route = routeFor(mode);
            break;

        case DriveMode::kAutomatic:
            if (isMotion(cmd, threshold) && takeover_mode != DriveMode::kAutomatic) {
                decision.route = routeFor(takeover_mode);
                decision.switch_to = takeover_mode;
            } else {
                decision.route = TeleopRoute::kDrop;
            }
            break;
    }

    return decision;
}

bool forwardGuarded(GuardedSource source, DriveMode mode)
{
    switch (source) {
        case GuardedSource::kTeleop: return mode == DriveMode::kAssisted;
        case GuardedSource::kNavigation: return mode == DriveMode::kAutomatic;
    }

    return false;
}

}  // namespace rover_drive_mode::domain
