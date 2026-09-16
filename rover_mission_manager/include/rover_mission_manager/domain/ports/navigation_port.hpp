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

#ifndef ROVER_MISSION_MANAGER_DOMAIN_PORTS_NAVIGATION_PORT_HPP_
#define ROVER_MISSION_MANAGER_DOMAIN_PORTS_NAVIGATION_PORT_HPP_

#include "rover_mission_manager/domain/mission.hpp"

namespace rover_mission_manager::domain::ports
{

/** @brief Outcome of the navigation request currently in flight. */
enum class NavigationResult
{
    kPending,   ///< Still driving.
    kReached,   ///< Arrived at the requested waypoint.
    kFailed,    ///< The navigator gave up or was rejected.
    kIdle,      ///< Nothing in flight.
};

/**
 * @brief Outbound port for driving the rover to a pose.
 *
 * Implemented in infrastructure by a Nav 2 NavigateToPose action client. Deliberately
 * non-blocking: goTo() dispatches and returns, and the use case polls result() on later
 * ticks, so nothing in the domain or application layer ever waits on I/O.
 */
class NavigationPort
{
public:
    virtual ~NavigationPort() = default;

    /**
     * @brief Start driving to a waypoint, replacing any goal in flight.
     * @return false when the goal could not be dispatched (navigator unreachable).
     */
    virtual bool goTo(const Waypoint & waypoint) = 0;

    /** @brief Abandon the goal in flight, if any. */
    virtual void cancel() = 0;

    /** @brief State of the goal in flight. */
    virtual NavigationResult result() const = 0;
};

}  // namespace rover_mission_manager::domain::ports

#endif  // ROVER_MISSION_MANAGER_DOMAIN_PORTS_NAVIGATION_PORT_HPP_
