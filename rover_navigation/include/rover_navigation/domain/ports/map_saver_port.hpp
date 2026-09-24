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

#include <functional>

#include "rover_navigation/domain/map_autosave_policy.hpp"

namespace rover_navigation::domain::ports
{

/**
 * @brief Outbound port for persisting the current map.
 *
 * Implemented in infrastructure by a nav2_msgs/srv/SaveMap client. Saving is asynchronous:
 * the saver is reached now, but the write succeeds or fails later.
 */
class MapSaverPort
{
public:
    virtual ~MapSaverPort() = default;

    /** @brief Called with whether the saver actually wrote the map. */
    using SaveDoneCallback = std::function<void(bool written)>;

    /**
     * @brief Request a map save.
     * @param on_done Called once the saver answers. Never called when this returns false,
     *        and may never be called if the saver goes silent.
     * @return false when the saver is unreachable, so the caller can back off.
     */
    virtual bool save(const MapSaveRequest & request, SaveDoneCallback on_done) = 0;
};

}  // namespace rover_navigation::domain::ports

#endif  // ROVER_NAVIGATION_DOMAIN_PORTS_MAP_SAVER_PORT_HPP_
