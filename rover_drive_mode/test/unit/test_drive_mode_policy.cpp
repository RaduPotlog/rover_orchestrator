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

#include "rover_drive_mode/domain/drive_mode.hpp"
#include "rover_drive_mode/domain/drive_mode_policy.hpp"

namespace rover_drive_mode::domain
{
namespace
{

Prerequisites allAvailable()
{
    Prerequisites prereqs;
    prereqs.mission_manager_available = true;
    prereqs.teleop_guard_available = true;
    return prereqs;
}

TEST(DriveModePolicyTest, ManualIsAlwaysReachable)
{
    const DriveModePolicy policy;
    Prerequisites nothing;
    nothing.teleop_guard_available = false;

    EXPECT_TRUE(policy.decide(DriveMode::kAutomatic, DriveMode::kManual, nothing).accepted);
    EXPECT_TRUE(policy.decide(DriveMode::kAssisted, DriveMode::kManual, nothing).accepted);
}

TEST(DriveModePolicyTest, AssistedNeedsItsGuardConfigured)
{
    const DriveModePolicy policy;
    auto prereqs = allAvailable();

    EXPECT_TRUE(policy.decide(DriveMode::kManual, DriveMode::kAssisted, prereqs).accepted);

    prereqs.teleop_guard_available = false;
    const auto decision = policy.decide(DriveMode::kManual, DriveMode::kAssisted, prereqs);
    EXPECT_FALSE(decision.accepted);
    EXPECT_FALSE(decision.reason.empty());
}

TEST(DriveModePolicyTest, AutomaticNeedsTheMissionManager)
{
    const DriveModePolicy policy;
    auto prereqs = allAvailable();

    EXPECT_TRUE(policy.decide(DriveMode::kAssisted, DriveMode::kAutomatic, prereqs).accepted);

    prereqs.mission_manager_available = false;
    const auto decision = policy.decide(DriveMode::kAssisted, DriveMode::kAutomatic, prereqs);
    EXPECT_FALSE(decision.accepted);
    EXPECT_NE(decision.reason.find("mission manager"), std::string::npos);
}

TEST(DriveModePolicyTest, AutomaticWithoutTheRequirementIgnoresTheMissionManager)
{
    const DriveModePolicy policy(false, 2.0);
    auto prereqs = allAvailable();
    prereqs.mission_manager_available = false;

    EXPECT_TRUE(policy.decide(DriveMode::kManual, DriveMode::kAutomatic, prereqs).accepted);
    EXPECT_FALSE(policy.forcedTransition(DriveMode::kAutomatic, prereqs, 100.0).has_value());
}

TEST(DriveModePolicyTest, RequestingTheCurrentModeIsAccepted)
{
    const DriveModePolicy policy;
    Prerequisites nothing;
    nothing.teleop_guard_available = false;

    // Even when its prerequisites are gone: staying put changes nothing.
    EXPECT_TRUE(policy.decide(DriveMode::kAutomatic, DriveMode::kAutomatic, nothing).accepted);
}

TEST(DriveModePolicyTest, FallbackIsAssistedWhenItsGuardIsAvailable)
{
    const DriveModePolicy policy;
    auto prereqs = allAvailable();

    EXPECT_EQ(policy.fallbackMode(prereqs), DriveMode::kAssisted);

    prereqs.teleop_guard_available = false;
    EXPECT_EQ(policy.fallbackMode(prereqs), DriveMode::kManual);
}

TEST(DriveModePolicyTest, LosingTheMissionManagerLeavesAutomaticAfterTheGrace)
{
    const DriveModePolicy policy(true, 2.0);
    auto prereqs = allAvailable();
    prereqs.mission_manager_available = false;

    EXPECT_FALSE(policy.forcedTransition(DriveMode::kAutomatic, prereqs, 1.9).has_value());

    const auto forced = policy.forcedTransition(DriveMode::kAutomatic, prereqs, 2.1);
    ASSERT_TRUE(forced.has_value());
    EXPECT_EQ(*forced, DriveMode::kAssisted);
}

TEST(DriveModePolicyTest, NoForcedTransitionOutsideAutomaticOrWithTheManagerPresent)
{
    const DriveModePolicy policy(true, 2.0);
    auto prereqs = allAvailable();

    EXPECT_FALSE(policy.forcedTransition(DriveMode::kAutomatic, prereqs, 10.0).has_value());

    prereqs.mission_manager_available = false;
    EXPECT_FALSE(policy.forcedTransition(DriveMode::kAssisted, prereqs, 10.0).has_value());
    EXPECT_FALSE(policy.forcedTransition(DriveMode::kManual, prereqs, 10.0).has_value());
}

TEST(DriveModeTest, ParsesParameterNames)
{
    EXPECT_EQ(driveModeFromString("manual"), DriveMode::kManual);
    EXPECT_EQ(driveModeFromString("assisted"), DriveMode::kAssisted);
    EXPECT_EQ(driveModeFromString("automatic"), DriveMode::kAutomatic);
    EXPECT_FALSE(driveModeFromString("MANUAL").has_value());
    EXPECT_FALSE(driveModeFromString("").has_value());
}

}  // namespace
}  // namespace rover_drive_mode::domain
