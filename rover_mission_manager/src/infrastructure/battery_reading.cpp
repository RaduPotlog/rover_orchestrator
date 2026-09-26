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

#include "rover_mission_manager/infrastructure/battery_reading.hpp"

#include <cmath>

namespace rover_mission_manager::infrastructure
{

std::optional<double> measuredBatteryFraction(const sensor_msgs::msg::BatteryState & msg)
{
    // rover_safety ignores UNKNOWN-status reports by the same rule. Older rover_battery builds
    // sent 0.0 in them, which aborted every running mission when the BMS watchdog expired.
    if (msg.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN ||
        !std::isfinite(msg.percentage))
    {
        return std::nullopt;
    }

    return static_cast<double>(msg.percentage);
}

}  // namespace rover_mission_manager::infrastructure
