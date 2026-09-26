// Copyright 2026 Mechatronics Academy
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

#ifndef ROVER_MISSION_MANAGER_INFRASTRUCTURE_BATTERY_READING_HPP_
#define ROVER_MISSION_MANAGER_INFRASTRUCTURE_BATTERY_READING_HPP_

#include <optional>

#include <sensor_msgs/msg/battery_state.hpp>

namespace rover_mission_manager::infrastructure
{

/**
 * @brief The state of charge a BatteryState message actually measured, as a 0..1 fraction.
 *
 * std::nullopt when the message carries no measurement: a NaN percentage (REP-0147's
 * "not measured"), or an UNKNOWN power-supply status, which is how rover_battery reports an
 * expired BMS watchdog. Callers keep their last good value instead -- a watchdog report must
 * never read as a flat pack.
 */
std::optional<double> measuredBatteryFraction(const sensor_msgs::msg::BatteryState & msg);

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_BATTERY_READING_HPP_
