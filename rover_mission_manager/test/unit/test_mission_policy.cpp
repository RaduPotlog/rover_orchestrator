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
using rover_mission_manager::domain::SensorHealth;

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

// --- Lidar health -----------------------------------------------------------------------
//
// rover_lidar's only health signal is a diagnostic_updater task, so these rules are what
// turns "the sensor feeding the costmaps died" into a decision the mission can act on.

TEST(MissionPolicyTest, HoldsWhenTheLidarIsUnhealthy)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.8;
    conditions.lidar_health = SensorHealth::kUnhealthy;

    // A hold, not an abort: the lidar can come back and the mission resumes on the same
    // waypoint, exactly as it does across a motion lock.
    EXPECT_EQ(policy.decide(conditions), MissionAction::kHold);
}

TEST(MissionPolicyTest, ProceedsWhenTheLidarIsHealthy)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.8;
    conditions.lidar_health = SensorHealth::kHealthy;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kProceed);
}

TEST(MissionPolicyTest, AnUnknownLidarProceedsByDefault)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.8;
    conditions.lidar_health = SensorHealth::kUnknown;

    // ROVER_USE_LIDAR=false is a supported configuration, and rover_lidar only starts after a
    // 10 s TimerAction when it is true. Failing closed here would make the mission unusable.
    EXPECT_EQ(policy.decide(conditions), MissionAction::kProceed);
}

TEST(MissionPolicyTest, AnUnknownLidarHoldsWhenTheLidarIsRequired)
{
    MissionPolicy policy(0.10, /*require_lidar=*/true);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.8;
    conditions.lidar_health = SensorHealth::kUnknown;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kHold);
    EXPECT_TRUE(policy.requireLidar());
}

TEST(MissionPolicyTest, AHealthyLidarStillProceedsWhenRequired)
{
    MissionPolicy policy(0.10, /*require_lidar=*/true);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.8;
    conditions.lidar_health = SensorHealth::kHealthy;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kProceed);
}

TEST(MissionPolicyTest, AFlatBatteryAbortsEvenWithADeadLidar)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.05;
    conditions.lidar_health = SensorHealth::kUnhealthy;

    // An abort outranks every hold, so the operator sees the real reason.
    EXPECT_EQ(policy.decide(conditions), MissionAction::kAbort);
}

TEST(MissionPolicyTest, ADeadLidarHoldsEvenWhenTheMotionLockIsAlsoEngaged)
{
    MissionPolicy policy(0.10);

    RoverConditions conditions;
    conditions.motion_locked = true;
    conditions.battery_fraction = 0.8;
    conditions.lidar_health = SensorHealth::kUnhealthy;

    EXPECT_EQ(policy.decide(conditions), MissionAction::kHold);
}

TEST(MissionPolicyTest, RequireLidarDefaultsToFalse)
{
    EXPECT_FALSE(MissionPolicy(0.10).requireLidar());
}
