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

#ifndef ROVER_MISSION_MANAGER_DOMAIN_MISSION_HPP_
#define ROVER_MISSION_MANAGER_DOMAIN_MISSION_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace rover_mission_manager::domain
{

/** @brief A pose the rover is asked to reach, in the mission's frame. */
struct Waypoint
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

/** @brief Where a mission is in its lifecycle. */
enum class MissionState
{
    kIdle,       ///< Nothing to do.
    kRunning,    ///< At least one waypoint still to reach.
    kHeldByLock, ///< Suspended because the motion lock is engaged.
    kSucceeded,  ///< Every waypoint reached.
    kFailed,     ///< Gave up (see Mission::failureReason).
    kCancelled,  ///< Cancelled by an operator.
};

/**
 * @brief An ordered list of waypoints plus the cursor into it.
 *
 * Pure data and pure transitions: no ROS, no clock, no I/O. The infrastructure layer maps
 * this onto Nav 2's NavigateToPose / NavigateThroughPoses actions.
 */
class Mission
{
public:
    Mission() = default;
    Mission(std::string id, std::vector<Waypoint> waypoints);

    const std::string & id() const { return id_; }
    MissionState state() const { return state_; }
    const std::vector<Waypoint> & waypoints() const { return waypoints_; }
    const std::string & failureReason() const { return failure_reason_; }

    /** @brief Index of the waypoint currently being driven to. */
    std::size_t currentIndex() const { return current_index_; }

    /** @brief The waypoint currently being driven to. @throws std::out_of_range when idle. */
    const Waypoint & currentWaypoint() const;

    bool isTerminal() const;

    /** @brief Begin the mission. An empty mission succeeds immediately. */
    void start();

    /** @brief Record that the current waypoint was reached; advances or succeeds. */
    void completeCurrentWaypoint();

    /** @brief Suspend on an engaged motion lock. No-op once terminal. */
    void hold();

    /** @brief Resume after hold(). No-op unless currently held. */
    void resume();

    void fail(std::string reason);
    void cancel();

private:
    std::string id_;
    std::vector<Waypoint> waypoints_;
    std::size_t current_index_ = 0;
    MissionState state_ = MissionState::kIdle;
    std::string failure_reason_;
};

/** @brief Stable, human-readable name for a state, for logging and status topics. */
const char * toString(MissionState state);

}  // namespace rover_mission_manager::domain

#endif  // ROVER_MISSION_MANAGER_DOMAIN_MISSION_HPP_
