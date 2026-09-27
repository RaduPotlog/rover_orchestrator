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

#include "rover_drive_mode/domain/drive_mode.hpp"

namespace rover_drive_mode::domain
{

const char * toString(DriveMode mode)
{
    switch (mode) {
        case DriveMode::kManual: return "MANUAL";
        case DriveMode::kAssisted: return "ASSISTED";
        case DriveMode::kAutomatic: return "AUTOMATIC";
    }

    return "UNKNOWN";
}

const char * toString(GuardState guard)
{
    switch (guard) {
        case GuardState::kBypassed: return "BYPASSED";
        case GuardState::kClear: return "CLEAR";
        case GuardState::kSlowing: return "SLOWING";
        case GuardState::kStopped: return "STOPPED";
        case GuardState::kNoData: return "NO_DATA";
    }

    return "UNKNOWN";
}

std::optional<DriveMode> driveModeFromString(const std::string & name)
{
    if (name == "manual") {
        return DriveMode::kManual;
    }

    if (name == "assisted") {
        return DriveMode::kAssisted;
    }

    if (name == "automatic") {
        return DriveMode::kAutomatic;
    }

    return std::nullopt;
}

}  // namespace rover_drive_mode::domain
