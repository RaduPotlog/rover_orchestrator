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

#include "rover_navigation/domain/map_autosave_policy.hpp"

#include <algorithm>

namespace rover_navigation::domain
{

constexpr std::chrono::duration<double> MapAutosavePolicy::kMinPeriod;

MapAutosavePolicy::MapAutosavePolicy(std::chrono::duration<double> requested_period)
: period_(requested_period),
  period_was_clamped_(false),
  consecutive_failures_(0),
  ticks_to_skip_(0)
{
    if (period_ < kMinPeriod) {
        period_ = kMinPeriod;
        period_was_clamped_ = true;
    }
}

int MapAutosavePolicy::currentBackoffTicks() const
{
    if (consecutive_failures_ <= 0) {
        return 0;
    }

    const int exponent = std::min(consecutive_failures_, kMaxBackoffExponent);
    return (1 << exponent) - 1;
}

bool MapAutosavePolicy::shouldSave()
{
    if (ticks_to_skip_ > 0) {
        --ticks_to_skip_;
        return false;
    }

    return true;
}

void MapAutosavePolicy::recordSuccess()
{
    consecutive_failures_ = 0;
    ticks_to_skip_ = 0;
}

void MapAutosavePolicy::recordFailure()
{
    ++consecutive_failures_;
    ticks_to_skip_ = currentBackoffTicks();
}

}  // namespace rover_navigation::domain
