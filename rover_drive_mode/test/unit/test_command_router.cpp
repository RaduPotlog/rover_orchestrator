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

#include "rover_drive_mode/domain/command_router.hpp"

namespace rover_drive_mode::domain
{
namespace
{

const TakeoverThreshold kThreshold{0.02, 0.02};
const Velocity kForward{0.5, 0.0};
const Velocity kCentred{0.0, 0.0};

TEST(CommandRouterTest, ManualGoesStraightToThePlatform)
{
    const auto decision = routeTeleop(DriveMode::kManual, kForward, kThreshold, DriveMode::kAssisted);
    EXPECT_EQ(decision.route, TeleopRoute::kDirect);
    EXPECT_FALSE(decision.switch_to.has_value());
}

TEST(CommandRouterTest, AssistedGoesThroughTheGuard)
{
    const auto decision =
        routeTeleop(DriveMode::kAssisted, kForward, kThreshold, DriveMode::kAssisted);
    EXPECT_EQ(decision.route, TeleopRoute::kGuard);
    EXPECT_FALSE(decision.switch_to.has_value());
}

TEST(CommandRouterTest, CentredStickIsForwardedOutsideAutomatic)
{
    // The UI's release burst of zeros is what stops the rover in MANUAL and ASSISTED.
    EXPECT_EQ(
        routeTeleop(DriveMode::kManual, kCentred, kThreshold, DriveMode::kAssisted).route,
        TeleopRoute::kDirect);
    EXPECT_EQ(
        routeTeleop(DriveMode::kAssisted, kCentred, kThreshold, DriveMode::kAssisted).route,
        TeleopRoute::kGuard);
}

TEST(CommandRouterTest, MovingStickInAutomaticTakesOverThroughTheGuard)
{
    const auto decision =
        routeTeleop(DriveMode::kAutomatic, kForward, kThreshold, DriveMode::kAssisted);
    EXPECT_EQ(decision.route, TeleopRoute::kGuard);
    ASSERT_TRUE(decision.switch_to.has_value());
    EXPECT_EQ(*decision.switch_to, DriveMode::kAssisted);
}

TEST(CommandRouterTest, TakeoverWithoutAGuardLandsInManual)
{
    const auto decision =
        routeTeleop(DriveMode::kAutomatic, Velocity{0.0, -0.5}, kThreshold, DriveMode::kManual);
    EXPECT_EQ(decision.route, TeleopRoute::kDirect);
    ASSERT_TRUE(decision.switch_to.has_value());
    EXPECT_EQ(*decision.switch_to, DriveMode::kManual);
}

TEST(CommandRouterTest, CentredOrTinyStickInAutomaticIsDropped)
{
    for (const auto & cmd : {kCentred, Velocity{0.02, 0.0}, Velocity{-0.01, 0.015}}) {
        const auto decision =
            routeTeleop(DriveMode::kAutomatic, cmd, kThreshold, DriveMode::kAssisted);
        EXPECT_EQ(decision.route, TeleopRoute::kDrop);
        EXPECT_FALSE(decision.switch_to.has_value());
    }
}

TEST(CommandRouterTest, EitherAxisAboveThresholdIsMotion)
{
    EXPECT_TRUE(isMotion(Velocity{0.03, 0.0}, kThreshold));
    EXPECT_TRUE(isMotion(Velocity{0.0, -0.03}, kThreshold));
    EXPECT_FALSE(isMotion(Velocity{0.02, -0.02}, kThreshold));
}

TEST(CommandRouterTest, GuardedCommandsPassOnlyInTheirOwnMode)
{
    EXPECT_TRUE(forwardGuarded(GuardedSource::kTeleop, DriveMode::kAssisted));
    EXPECT_FALSE(forwardGuarded(GuardedSource::kTeleop, DriveMode::kManual));
    EXPECT_FALSE(forwardGuarded(GuardedSource::kTeleop, DriveMode::kAutomatic));

    EXPECT_TRUE(forwardGuarded(GuardedSource::kNavigation, DriveMode::kAutomatic));
    EXPECT_FALSE(forwardGuarded(GuardedSource::kNavigation, DriveMode::kManual));
    EXPECT_FALSE(forwardGuarded(GuardedSource::kNavigation, DriveMode::kAssisted));
}

}  // namespace
}  // namespace rover_drive_mode::domain
