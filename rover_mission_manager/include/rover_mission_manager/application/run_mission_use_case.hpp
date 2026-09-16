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

#ifndef ROVER_MISSION_MANAGER_APPLICATION_RUN_MISSION_USE_CASE_HPP_
#define ROVER_MISSION_MANAGER_APPLICATION_RUN_MISSION_USE_CASE_HPP_

#include <cstddef>
#include <memory>

#include "rover_mission_manager/domain/mission.hpp"
#include "rover_mission_manager/domain/mission_policy.hpp"
#include "rover_mission_manager/domain/ports/mission_status_publisher_port.hpp"
#include "rover_mission_manager/domain/ports/navigation_port.hpp"

namespace rover_mission_manager::application
{

/**
 * @brief Drives one mission to completion, one tick at a time.
 *
 * Owns the interaction between the mission's bookkeeping (domain::Mission), the safety rules
 * (domain::MissionPolicy) and the navigator (domain::ports::NavigationPort). Every call
 * returns promptly; the caller supplies the cadence.
 */
class RunMissionUseCase
{
public:
    RunMissionUseCase(
        std::shared_ptr<domain::ports::NavigationPort> navigation,
        std::shared_ptr<domain::ports::MissionStatusPublisherPort> status_publisher,
        domain::MissionPolicy policy);

    /** @brief Replace the active mission and start it. Cancels anything in flight. */
    void accept(domain::Mission mission);

    /** @brief Cancel the active mission and stop the rover. */
    void cancel();

    /**
     * @brief Advance the active mission by one tick.
     * @param conditions The rover's current conditions, sampled by the caller.
     */
    void tick(const domain::RoverConditions & conditions);

    const domain::Mission & mission() const { return mission_; }

private:
    /** @brief Dispatch the current waypoint if nothing is in flight. */
    void driveCurrentWaypoint();

    void publishStatus();

    std::shared_ptr<domain::ports::NavigationPort> navigation_;
    std::shared_ptr<domain::ports::MissionStatusPublisherPort> status_publisher_;
    domain::MissionPolicy policy_;

    domain::Mission mission_;
    domain::MissionState last_published_state_;
    std::size_t last_published_index_;
    bool goal_in_flight_;
};

}  // namespace rover_mission_manager::application

#endif  // ROVER_MISSION_MANAGER_APPLICATION_RUN_MISSION_USE_CASE_HPP_
