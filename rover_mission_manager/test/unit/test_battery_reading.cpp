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

#include <limits>

#include <gtest/gtest.h>

#include "rover_mission_manager/infrastructure/battery_reading.hpp"

using rover_mission_manager::infrastructure::measuredBatteryFraction;
using sensor_msgs::msg::BatteryState;

namespace
{

BatteryState report(std::uint8_t status, float percentage)
{
    BatteryState msg;
    msg.power_supply_status = status;
    msg.percentage = percentage;
    return msg;
}

}  // namespace

TEST(MeasuredBatteryFraction, PassesThroughAMeasuredPercentage)
{
    for (const auto status : {BatteryState::POWER_SUPPLY_STATUS_CHARGING,
             BatteryState::POWER_SUPPLY_STATUS_DISCHARGING,
             BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING,
             BatteryState::POWER_SUPPLY_STATUS_FULL})
    {
        const auto fraction = measuredBatteryFraction(report(status, 0.41f));
        ASSERT_TRUE(fraction.has_value()) << "status " << unsigned(status);
        EXPECT_NEAR(*fraction, 0.41, 1e-6);
    }
}

TEST(MeasuredBatteryFraction, KeepsAGenuinelyEmptyPack)
{
    // A measured 0 % must still reach the abort rule.
    const auto fraction =
        measuredBatteryFraction(report(BatteryState::POWER_SUPPLY_STATUS_DISCHARGING, 0.0f));
    ASSERT_TRUE(fraction.has_value());
    EXPECT_DOUBLE_EQ(*fraction, 0.0);
}

TEST(MeasuredBatteryFraction, IgnoresAWatchdogReportFromOlderRoverBattery)
{
    // What rover_battery published on BMS-watchdog expiry before percentage became NaN.
    EXPECT_FALSE(
        measuredBatteryFraction(report(BatteryState::POWER_SUPPLY_STATUS_UNKNOWN, 0.0f)));
}

TEST(MeasuredBatteryFraction, IgnoresAnUnmeasuredPercentage)
{
    EXPECT_FALSE(measuredBatteryFraction(report(
        BatteryState::POWER_SUPPLY_STATUS_UNKNOWN, std::numeric_limits<float>::quiet_NaN())));
    EXPECT_FALSE(measuredBatteryFraction(report(
        BatteryState::POWER_SUPPLY_STATUS_DISCHARGING, std::numeric_limits<float>::quiet_NaN())));
    EXPECT_FALSE(measuredBatteryFraction(report(
        BatteryState::POWER_SUPPLY_STATUS_DISCHARGING, std::numeric_limits<float>::infinity())));
}
