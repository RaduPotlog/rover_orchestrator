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

#include "rover_navigation/application/autosave_map_use_case.hpp"

#include <stdexcept>
#include <utility>

namespace rover_navigation::application
{

AutosaveMapUseCase::AutosaveMapUseCase(
    std::shared_ptr<domain::ports::MapSaverPort> map_saver,
    domain::MapAutosavePolicy policy,
    domain::MapSaveRequest request)
: map_saver_(std::move(map_saver)), policy_(policy), request_(std::move(request))
{
    if (map_saver_ == nullptr) {
        throw std::invalid_argument("AutosaveMapUseCase requires a MapSaverPort");
    }
}

constexpr int AutosaveMapUseCase::kMaxTicksInFlight;

AutosaveOutcome AutosaveMapUseCase::execute()
{
    if (in_flight_) {
        if (++ticks_in_flight_ < kMaxTicksInFlight) {
            return AutosaveOutcome::kSkippedInFlight;
        }

        in_flight_ = false;
        policy_.recordFailure();
        return AutosaveOutcome::kSaveTimedOut;
    }

    if (!policy_.shouldSave()) {
        return AutosaveOutcome::kSkippedBackingOff;
    }

    // Marked in flight before dispatching, in case the saver answers from inside save().
    const unsigned int generation = ++generation_;
    in_flight_ = true;
    ticks_in_flight_ = 0;

    const bool dispatched = map_saver_->save(
        request_, [this, generation](bool written) { onSaveDone(generation, written); });

    if (!dispatched) {
        in_flight_ = false;
        policy_.recordFailure();
        return AutosaveOutcome::kSaverUnavailable;
    }

    return AutosaveOutcome::kRequested;
}

void AutosaveMapUseCase::onSaveDone(unsigned int generation, bool written)
{
    if (!in_flight_ || generation != generation_) {
        return;
    }

    in_flight_ = false;

    if (written) {
        policy_.recordSuccess();
    } else {
        policy_.recordFailure();
    }
}

}  // namespace rover_navigation::application
