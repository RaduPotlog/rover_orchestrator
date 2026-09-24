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

#ifndef ROVER_NAVIGATION_DOMAIN_PORTS_MAP_SAVER_PORT_HPP_
#define ROVER_NAVIGATION_DOMAIN_PORTS_MAP_SAVER_PORT_HPP_

#include "rover_navigation/domain/map_autosave_policy.hpp"

namespace rover_navigation::domain::ports
{

/**
 * @brief Outbound port for persisting the current map.
 *
 * Implemented in infrastructure by a nav2_msgs/srv/SaveMap client. The use case only needs
 * to know whether the request could be dispatched, not how.
 */
class MapSaverPort
{
public:
    virtual ~MapSaverPort() = default;

    /**
     * @brief Request a map save.
     * @return false when the saver is unreachable, so the caller can back off.
     */
    virtual bool save(const MapSaveRequest & request) = 0;
};

}  // namespace rover_navigation::domain::ports

#endif  // ROVER_NAVIGATION_DOMAIN_PORTS_MAP_SAVER_PORT_HPP_
