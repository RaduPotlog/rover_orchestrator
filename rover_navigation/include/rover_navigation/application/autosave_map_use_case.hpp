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

#ifndef ROVER_NAVIGATION_APPLICATION_AUTOSAVE_MAP_USE_CASE_HPP_
#define ROVER_NAVIGATION_APPLICATION_AUTOSAVE_MAP_USE_CASE_HPP_

#include <memory>

#include "rover_navigation/domain/map_autosave_policy.hpp"
#include "rover_navigation/domain/ports/map_saver_port.hpp"

namespace rover_navigation::application
{

/** @brief What one tick of the autosaver did, so the adapter can log it. */
enum class AutosaveOutcome
{
    kSaved,
    kSkippedBackingOff,
    kSaverUnavailable,
};

/** @brief Periodically asks the map saver to persist the map, backing off on failure. */
class AutosaveMapUseCase
{
public:
    AutosaveMapUseCase(
        std::shared_ptr<domain::ports::MapSaverPort> map_saver,
        domain::MapAutosavePolicy policy,
        domain::MapSaveRequest request);

    /** @brief Run one autosave tick. */
    AutosaveOutcome execute();

    const domain::MapAutosavePolicy & policy() const { return policy_; }

private:
    std::shared_ptr<domain::ports::MapSaverPort> map_saver_;
    domain::MapAutosavePolicy policy_;
    domain::MapSaveRequest request_;
};

}  // namespace rover_navigation::application

#endif  // ROVER_NAVIGATION_APPLICATION_AUTOSAVE_MAP_USE_CASE_HPP_
