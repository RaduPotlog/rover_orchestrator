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

#ifndef ROVER_MISSION_MANAGER_DOMAIN_PORTS_MISSION_STATUS_PUBLISHER_PORT_HPP_
#define ROVER_MISSION_MANAGER_DOMAIN_PORTS_MISSION_STATUS_PUBLISHER_PORT_HPP_

#include "rover_mission_manager/domain/mission.hpp"

namespace rover_mission_manager::domain::ports
{

/**
 * @brief Outbound port for reporting mission progress to the rest of the system.
 *
 * rover_msgs has no mission message today, so the ROS adapter publishes a std_msgs/String.
 * Keeping it behind a port means swapping in a typed message later does not touch the
 * application layer.
 */
class MissionStatusPublisherPort
{
public:
    virtual ~MissionStatusPublisherPort() = default;

    virtual void publish(const Mission & mission) = 0;
};

}  // namespace rover_mission_manager::domain::ports

#endif  // ROVER_MISSION_MANAGER_DOMAIN_PORTS_MISSION_STATUS_PUBLISHER_PORT_HPP_
