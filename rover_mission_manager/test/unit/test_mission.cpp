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

#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "rover_mission_manager/domain/mission.hpp"

using rover_mission_manager::domain::Mission;
using rover_mission_manager::domain::MissionState;
using rover_mission_manager::domain::Waypoint;

namespace
{
Mission twoWaypointMission()
{
    return Mission("m1", {Waypoint{1.0, 0.0, 0.0}, Waypoint{2.0, 1.0, 1.57}});
}
}  // namespace

TEST(MissionTest, StartsIdle)
{
    Mission mission = twoWaypointMission();

    EXPECT_EQ(mission.state(), MissionState::kIdle);
    EXPECT_FALSE(mission.isTerminal());
}

TEST(MissionTest, RunsThroughItsWaypointsInOrder)
{
    Mission mission = twoWaypointMission();
    mission.start();

    ASSERT_EQ(mission.state(), MissionState::kRunning);
    EXPECT_DOUBLE_EQ(mission.currentWaypoint().x, 1.0);

    mission.completeCurrentWaypoint();
    ASSERT_EQ(mission.state(), MissionState::kRunning);
    EXPECT_DOUBLE_EQ(mission.currentWaypoint().x, 2.0);

    mission.completeCurrentWaypoint();
    EXPECT_EQ(mission.state(), MissionState::kSucceeded);
    EXPECT_TRUE(mission.isTerminal());
}

TEST(MissionTest, AnEmptyMissionSucceedsImmediatelyRatherThanRunningWithNoWaypoint)
{
    Mission mission("empty", {});
    mission.start();

    EXPECT_EQ(mission.state(), MissionState::kSucceeded);
    EXPECT_THROW((void)mission.currentWaypoint(), std::out_of_range);
}

TEST(MissionTest, HoldAndResumeArePairedAndDoNotLoseProgress)
{
    Mission mission = twoWaypointMission();
    mission.start();
    mission.completeCurrentWaypoint();

    mission.hold();
    EXPECT_EQ(mission.state(), MissionState::kHeldByLock);
    EXPECT_EQ(mission.currentIndex(), 1u);

    mission.resume();
    EXPECT_EQ(mission.state(), MissionState::kRunning);
    EXPECT_EQ(mission.currentIndex(), 1u);
}

TEST(MissionTest, CompletingAWaypointWhileHeldIsIgnored)
{
    Mission mission = twoWaypointMission();
    mission.start();
    mission.hold();

    mission.completeCurrentWaypoint();

    EXPECT_EQ(mission.state(), MissionState::kHeldByLock);
    EXPECT_EQ(mission.currentIndex(), 0u);
}

TEST(MissionTest, FailureRecordsItsReason)
{
    Mission mission = twoWaypointMission();
    mission.start();
    mission.fail("navigation failed");

    EXPECT_EQ(mission.state(), MissionState::kFailed);
    EXPECT_EQ(mission.failureReason(), "navigation failed");
}

TEST(MissionTest, ATerminalMissionCannotBeReopened)
{
    Mission mission = twoWaypointMission();
    mission.start();
    mission.cancel();

    ASSERT_EQ(mission.state(), MissionState::kCancelled);

    mission.fail("too late");
    mission.hold();
    mission.completeCurrentWaypoint();

    EXPECT_EQ(mission.state(), MissionState::kCancelled);
    EXPECT_TRUE(mission.failureReason().empty());
}

TEST(MissionTest, StateNamesAreStable)
{
    EXPECT_STREQ(toString(MissionState::kIdle), "IDLE");
    EXPECT_STREQ(toString(MissionState::kRunning), "RUNNING");
    EXPECT_STREQ(toString(MissionState::kHeldByLock), "HELD_BY_LOCK");
    EXPECT_STREQ(toString(MissionState::kSucceeded), "SUCCEEDED");
    EXPECT_STREQ(toString(MissionState::kFailed), "FAILED");
    EXPECT_STREQ(toString(MissionState::kCancelled), "CANCELLED");
}
