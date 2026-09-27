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

#ifndef ROVER_DRIVE_MODE_DOMAIN_PORTS_DRIVE_MODE_OUTPUT_PORT_HPP_
#define ROVER_DRIVE_MODE_DOMAIN_PORTS_DRIVE_MODE_OUTPUT_PORT_HPP_

#include <string>

#include "rover_drive_mode/domain/drive_mode.hpp"

namespace rover_drive_mode::domain::ports
{

/** @brief The mode as the rest of the rover sees it. */
struct DriveModeStatus
{
    DriveMode mode = DriveMode::kAssisted;
    GuardState guard = GuardState::kNoData;
    std::string reason;
};

/** @brief Side effects of the drive-mode use case. */
class DriveModeOutputPort
{
public:
    virtual ~DriveModeOutputPort() = default;

    /** @brief Announce the current mode and guard state (latched for late joiners). */
    virtual void publishStatus(const DriveModeStatus & status) = 0;

    /**
     * @brief Send one zero command on the navigation input of the platform.
     *
     * Called on leaving AUTOMATIC: the mux keeps a silent input's last command until its
     * timeout, so without it the rover would coast on Nav 2's last velocity for that long.
     */
    virtual void stopNavigation() = 0;
};

}  // namespace rover_drive_mode::domain::ports

#endif  // ROVER_DRIVE_MODE_DOMAIN_PORTS_DRIVE_MODE_OUTPUT_PORT_HPP_
