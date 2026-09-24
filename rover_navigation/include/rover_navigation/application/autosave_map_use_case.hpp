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
    kRequested,
    kSkippedBackingOff,
    kSkippedInFlight,
    kSaverUnavailable,
    kSaveTimedOut,
};

/**
 * @brief Periodically asks the map saver to persist the map, backing off on failure.
 *
 * A save counts as a success only once the saver reports the map written. A saver that is
 * unreachable, fails the write, or never answers all count as failures and back off alike.
 * Only one request is outstanding at a time.
 */
class AutosaveMapUseCase
{
public:
    /** @brief Ticks a request may stay unanswered before it counts as a failure. */
    static constexpr int kMaxTicksInFlight = 3;

    AutosaveMapUseCase(
        std::shared_ptr<domain::ports::MapSaverPort> map_saver,
        domain::MapAutosavePolicy policy,
        domain::MapSaveRequest request);

    // Pending save callbacks hold `this`, so the use case must stay put.
    AutosaveMapUseCase(const AutosaveMapUseCase &) = delete;
    AutosaveMapUseCase & operator=(const AutosaveMapUseCase &) = delete;

    /** @brief Run one autosave tick. */
    AutosaveOutcome execute();

    const domain::MapAutosavePolicy & policy() const { return policy_; }

private:
    void onSaveDone(unsigned int generation, bool written);

    std::shared_ptr<domain::ports::MapSaverPort> map_saver_;
    domain::MapAutosavePolicy policy_;
    domain::MapSaveRequest request_;

    bool in_flight_ = false;
    int ticks_in_flight_ = 0;
    // Bumped per request, so a reply that arrives after its request timed out is ignored.
    unsigned int generation_ = 0;
};

}  // namespace rover_navigation::application

#endif  // ROVER_NAVIGATION_APPLICATION_AUTOSAVE_MAP_USE_CASE_HPP_
