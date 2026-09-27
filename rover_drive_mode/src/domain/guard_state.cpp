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

#include "rover_drive_mode/domain/guard_state.hpp"

namespace rover_drive_mode::domain
{

namespace
{

GuardState fromReport(const GuardReport & report)
{
    if (!report.running || report.source_invalid) {
        return GuardState::kNoData;
    }

    if (!report.received) {
        return GuardState::kClear;
    }

    switch (report.action) {
        case CollisionAction::kStop: return GuardState::kStopped;
        case CollisionAction::kSlowdown:
        case CollisionAction::kLimit:
        case CollisionAction::kApproach: return GuardState::kSlowing;
        case CollisionAction::kDoNothing: return GuardState::kClear;
    }

    return GuardState::kClear;
}

}  // namespace

GuardState guardState(
    DriveMode mode, const GuardReport & teleop_guard, const GuardReport & navigation_guard)
{
    switch (mode) {
        case DriveMode::kManual: return GuardState::kBypassed;
        case DriveMode::kAssisted: return fromReport(teleop_guard);
        case DriveMode::kAutomatic: return fromReport(navigation_guard);
    }

    return GuardState::kNoData;
}

}  // namespace rover_drive_mode::domain
