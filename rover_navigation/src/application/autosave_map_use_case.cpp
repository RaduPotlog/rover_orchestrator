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

AutosaveOutcome AutosaveMapUseCase::execute()
{
    if (!policy_.shouldSave()) {
        return AutosaveOutcome::kSkippedBackingOff;
    }

    if (!map_saver_->save(request_)) {
        policy_.recordFailure();
        return AutosaveOutcome::kSaverUnavailable;
    }

    policy_.recordSuccess();
    return AutosaveOutcome::kSaved;
}

}  // namespace rover_navigation::application
