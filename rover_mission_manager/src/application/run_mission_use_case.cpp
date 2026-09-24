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

#include "rover_mission_manager/application/run_mission_use_case.hpp"

#include <stdexcept>
#include <utility>

namespace rover_mission_manager::application
{

using domain::MissionAction;
using domain::MissionState;
using domain::ports::DispatchResult;
using domain::ports::NavigationResult;

RunMissionUseCase::RunMissionUseCase(
    std::shared_ptr<domain::ports::NavigationPort> navigation,
    std::shared_ptr<domain::ports::MissionStatusPublisherPort> status_publisher,
    domain::MissionPolicy policy)
: navigation_(std::move(navigation)),
  status_publisher_(std::move(status_publisher)),
  policy_(policy),
  last_published_state_(MissionState::kIdle),
  last_published_index_(0),
  goal_in_flight_(false)
{
    if (navigation_ == nullptr) {
        throw std::invalid_argument("RunMissionUseCase requires a NavigationPort");
    }

    if (status_publisher_ == nullptr) {
        throw std::invalid_argument("RunMissionUseCase requires a MissionStatusPublisherPort");
    }
}

void RunMissionUseCase::accept(domain::Mission mission)
{
    navigation_->cancel();
    goal_in_flight_ = false;

    mission_ = std::move(mission);
    mission_.start();

    publishStatus();
}

void RunMissionUseCase::cancel()
{
    navigation_->cancel();
    goal_in_flight_ = false;

    mission_.cancel();

    publishStatus();
}

void RunMissionUseCase::driveCurrentWaypoint()
{
    if (goal_in_flight_) {
        return;
    }

    switch (navigation_->goTo(mission_.currentWaypoint())) {
        case DispatchResult::kDispatched:
            goal_in_flight_ = true;
            break;

        case DispatchResult::kNotReady:
            // Nothing in flight: the next tick sees kIdle and tries again.
            break;

        case DispatchResult::kUnreachable:
            mission_.fail("navigator unreachable");
            break;
    }
}

void RunMissionUseCase::tick(const domain::RoverConditions & conditions)
{
    if (mission_.isTerminal() || mission_.state() == MissionState::kIdle) {
        publishStatus();
        return;
    }

    switch (policy_.decide(conditions)) {
        case MissionAction::kAbort:
            navigation_->cancel();
            goal_in_flight_ = false;
            mission_.fail("aborted by mission policy");
            publishStatus();
            return;

        case MissionAction::kHold:
            // Drop the goal rather than letting Nav 2 keep planning against a closed mux.
            // The waypoint is re-dispatched from the mission cursor once the lock clears.
            if (goal_in_flight_) {
                navigation_->cancel();
                goal_in_flight_ = false;
            }
            mission_.hold();
            publishStatus();
            return;

        case MissionAction::kProceed:
            mission_.resume();
            break;
    }

    switch (navigation_->result()) {
        case NavigationResult::kReached:
            goal_in_flight_ = false;
            mission_.completeCurrentWaypoint();

            if (!mission_.isTerminal()) {
                driveCurrentWaypoint();
            }
            break;

        case NavigationResult::kFailed:
            goal_in_flight_ = false;
            mission_.fail("navigation failed");
            break;

        case NavigationResult::kIdle:
            driveCurrentWaypoint();
            break;

        case NavigationResult::kPending:
            break;
    }

    publishStatus();
}

void RunMissionUseCase::publishStatus()
{
    // Only on a transition -- a new state, or progress onto the next waypoint. The manager
    // ticks at tens of hertz and the status topic is for operators, not for control.
    if (mission_.state() == last_published_state_ &&
        mission_.currentIndex() == last_published_index_)
    {
        return;
    }

    last_published_state_ = mission_.state();
    last_published_index_ = mission_.currentIndex();
    status_publisher_->publish(mission_);
}

}  // namespace rover_mission_manager::application
