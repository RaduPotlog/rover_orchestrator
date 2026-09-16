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

#include "rover_mission_manager/domain/mission.hpp"

#include <stdexcept>
#include <utility>

namespace rover_mission_manager::domain
{

Mission::Mission(std::string id, std::vector<Waypoint> waypoints)
: id_(std::move(id)), waypoints_(std::move(waypoints))
{
}

const Waypoint & Mission::currentWaypoint() const
{
    if (current_index_ >= waypoints_.size()) {
        throw std::out_of_range("Mission has no current waypoint");
    }

    return waypoints_[current_index_];
}

bool Mission::isTerminal() const
{
    return state_ == MissionState::kSucceeded || state_ == MissionState::kFailed ||
           state_ == MissionState::kCancelled;
}

void Mission::start()
{
    current_index_ = 0;
    failure_reason_.clear();

    // A mission with nothing to drive to is vacuously complete; it must not sit in kRunning
    // with no current waypoint, or currentWaypoint() would throw on the next tick.
    state_ = waypoints_.empty() ? MissionState::kSucceeded : MissionState::kRunning;
}

void Mission::completeCurrentWaypoint()
{
    if (state_ != MissionState::kRunning) {
        return;
    }

    ++current_index_;

    if (current_index_ >= waypoints_.size()) {
        state_ = MissionState::kSucceeded;
    }
}

void Mission::hold()
{
    if (state_ == MissionState::kRunning) {
        state_ = MissionState::kHeldByLock;
    }
}

void Mission::resume()
{
    if (state_ == MissionState::kHeldByLock) {
        state_ = MissionState::kRunning;
    }
}

void Mission::fail(std::string reason)
{
    if (isTerminal()) {
        return;
    }

    state_ = MissionState::kFailed;
    failure_reason_ = std::move(reason);
}

void Mission::cancel()
{
    if (isTerminal()) {
        return;
    }

    state_ = MissionState::kCancelled;
}

const char * toString(MissionState state)
{
    switch (state) {
        case MissionState::kIdle:
            return "IDLE";
        case MissionState::kRunning:
            return "RUNNING";
        case MissionState::kHeldByLock:
            return "HELD_BY_LOCK";
        case MissionState::kSucceeded:
            return "SUCCEEDED";
        case MissionState::kFailed:
            return "FAILED";
        case MissionState::kCancelled:
            return "CANCELLED";
    }

    return "UNKNOWN";
}

}  // namespace rover_mission_manager::domain
