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

#include <gtest/gtest.h>

#include "rover_mission_manager/domain/mission_policy.hpp"

using rover_mission_manager::domain::MissionAction;
using rover_mission_manager::domain::MissionPolicy;
using rover_mission_manager::domain::RoverConditions;

TEST(MissionPolicyTest, ProceedsWhenUnlockedAndCharged)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.8;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kProceed);
}

TEST(MissionPolicyTest, HoldsWhileTheMotionLockIsEngaged)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = true;
    conditions.battery_fraction = 0.8;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kHold);
}

TEST(MissionPolicyTest, DefaultConditionsAreFailSafe)
{
    MissionPolicy policy(0.10);

    // A default-constructed RoverConditions means "nothing heard yet", which must not drive.
    EXPECT_EQ(policy.decide(RoverConditions{}), MissionAction::kHold);
}

TEST(MissionPolicyTest, AbortsOnAFlatBattery)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.05;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kAbort);
}

TEST(MissionPolicyTest, AFlatBatteryAbortsEvenWhileLocked)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = true;
    conditions.battery_fraction = 0.05;

    // Holding would keep discharging, and the lock must not hide why the mission stopped.
    EXPECT_EQ(policy.decide(conditions), MissionAction::kAbort);
}

TEST(MissionPolicyTest, AnUnknownBatteryIsNotTreatedAsFlat)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = -1.0;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kProceed);
}

TEST(MissionPolicyTest, ClampsAnOutOfRangeThreshold)
{
    EXPECT_DOUBLE_EQ(MissionPolicy(5.0).abortBatteryFraction(), 1.0);
    EXPECT_DOUBLE_EQ(MissionPolicy(-2.0).abortBatteryFraction(), 0.0);
}
