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

#include <gtest/gtest.h>

#include "rover_drive_mode/domain/guard_state.hpp"

namespace rover_drive_mode::domain
{
namespace
{

GuardReport running(CollisionAction action)
{
    GuardReport report;
    report.running = true;
    report.received = true;
    report.action = action;
    return report;
}

TEST(GuardStateTest, ManualIsAlwaysBypassed)
{
    EXPECT_EQ(guardState(DriveMode::kManual, GuardReport{}, GuardReport{}), GuardState::kBypassed);
    EXPECT_EQ(
        guardState(
            DriveMode::kManual, running(CollisionAction::kStop), running(CollisionAction::kStop)),
        GuardState::kBypassed);
}

TEST(GuardStateTest, EachModeReadsItsOwnMonitor)
{
    const auto stopped = running(CollisionAction::kStop);
    const auto slowing = running(CollisionAction::kSlowdown);

    EXPECT_EQ(guardState(DriveMode::kAssisted, stopped, slowing), GuardState::kStopped);
    EXPECT_EQ(guardState(DriveMode::kAutomatic, stopped, slowing), GuardState::kSlowing);
}

TEST(GuardStateTest, MonitorNotRunningIsNoData)
{
    EXPECT_EQ(guardState(DriveMode::kAssisted, GuardReport{}, GuardReport{}), GuardState::kNoData);
}

TEST(GuardStateTest, RunningMonitorWithoutReportsIsClear)
{
    // nav2_collision_monitor only reports zone changes; silence means nothing entered a zone.
    GuardReport report;
    report.running = true;
    EXPECT_EQ(guardState(DriveMode::kAssisted, report, GuardReport{}), GuardState::kClear);
}

TEST(GuardStateTest, StaleLidarIsNoDataNotAnObstacle)
{
    auto report = running(CollisionAction::kStop);
    report.source_invalid = true;
    EXPECT_EQ(guardState(DriveMode::kAssisted, report, GuardReport{}), GuardState::kNoData);
}

TEST(GuardStateTest, SpeedReducingActionsAreSlowing)
{
    for (const auto action :
         {CollisionAction::kSlowdown, CollisionAction::kLimit, CollisionAction::kApproach})
    {
        EXPECT_EQ(
            guardState(DriveMode::kAssisted, running(action), GuardReport{}),
            GuardState::kSlowing);
    }

    EXPECT_EQ(
        guardState(DriveMode::kAssisted, running(CollisionAction::kDoNothing), GuardReport{}),
        GuardState::kClear);
}

}  // namespace
}  // namespace rover_drive_mode::domain
