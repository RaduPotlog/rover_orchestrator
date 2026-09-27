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

#include <memory>
#include <vector>

#include "rover_drive_mode/application/drive_mode_use_case.hpp"

namespace rover_drive_mode::application
{
namespace
{

using domain::DriveMode;
using domain::GuardState;
using domain::ports::DriveModeStatus;

class FakeOutput : public domain::ports::DriveModeOutputPort
{
public:
    void publishStatus(const DriveModeStatus & status) override { published.push_back(status); }
    void stopNavigation() override { ++navigation_stops; }

    std::vector<DriveModeStatus> published;
    int navigation_stops = 0;
};

domain::Prerequisites allAvailable()
{
    domain::Prerequisites prereqs;
    prereqs.mission_manager_available = true;
    prereqs.teleop_guard_available = true;
    return prereqs;
}

domain::GuardReport runningGuard()
{
    domain::GuardReport report;
    report.running = true;
    return report;
}

class DriveModeUseCaseTest : public ::testing::Test
{
protected:
    void make(DriveMode initial, const domain::Prerequisites & prereqs = allAvailable())
    {
        output_ = std::make_shared<FakeOutput>();
        use_case_ = std::make_unique<DriveModeUseCase>(
            output_, domain::DriveModePolicy(true, 2.0), domain::TakeoverThreshold{},
            initial, "boot default");
        use_case_->onGuardReports(runningGuard(), runningGuard());
        use_case_->start(prereqs);
    }

    std::shared_ptr<FakeOutput> output_;
    std::unique_ptr<DriveModeUseCase> use_case_;
};

TEST_F(DriveModeUseCaseTest, PublishesTheBootModeOnStart)
{
    make(DriveMode::kAssisted);

    ASSERT_FALSE(output_->published.empty());
    EXPECT_EQ(output_->published.back().mode, DriveMode::kAssisted);
    EXPECT_EQ(output_->published.back().guard, GuardState::kClear);
    EXPECT_EQ(output_->published.back().reason, "boot default");
}

TEST_F(DriveModeUseCaseTest, BootsInManualWhenAssistedIsUnavailable)
{
    auto prereqs = allAvailable();
    prereqs.teleop_guard_available = false;
    make(DriveMode::kAssisted, prereqs);

    EXPECT_EQ(use_case_->mode(), DriveMode::kManual);
    EXPECT_EQ(use_case_->guard(), GuardState::kBypassed);
}

TEST_F(DriveModeUseCaseTest, AcceptedRequestChangesAndPublishesTheMode)
{
    make(DriveMode::kAssisted);

    const auto decision = use_case_->request(DriveMode::kManual, allAvailable());

    EXPECT_TRUE(decision.accepted);
    EXPECT_EQ(use_case_->mode(), DriveMode::kManual);
    EXPECT_EQ(output_->published.back().mode, DriveMode::kManual);
    EXPECT_EQ(output_->published.back().guard, GuardState::kBypassed);
    EXPECT_EQ(output_->published.back().reason, "operator request");
}

TEST_F(DriveModeUseCaseTest, RefusedRequestKeepsTheMode)
{
    make(DriveMode::kAssisted);
    const auto published_before = output_->published.size();

    auto prereqs = allAvailable();
    prereqs.mission_manager_available = false;
    const auto decision = use_case_->request(DriveMode::kAutomatic, prereqs);

    EXPECT_FALSE(decision.accepted);
    EXPECT_EQ(use_case_->mode(), DriveMode::kAssisted);
    EXPECT_EQ(output_->published.size(), published_before);
}

TEST_F(DriveModeUseCaseTest, TakeoverSwitchesToAssistedAndStopsNavigation)
{
    make(DriveMode::kAssisted);
    use_case_->request(DriveMode::kAutomatic, allAvailable());

    const auto route = use_case_->onTeleop(domain::Velocity{0.4, 0.0}, allAvailable());

    EXPECT_EQ(route, domain::TeleopRoute::kGuard);
    EXPECT_EQ(use_case_->mode(), DriveMode::kAssisted);
    EXPECT_EQ(use_case_->reason(), "operator takeover");
    EXPECT_EQ(output_->navigation_stops, 1);
}

TEST_F(DriveModeUseCaseTest, CentredStickInAutomaticChangesNothing)
{
    make(DriveMode::kAssisted);
    use_case_->request(DriveMode::kAutomatic, allAvailable());

    const auto route = use_case_->onTeleop(domain::Velocity{0.0, 0.0}, allAvailable());

    EXPECT_EQ(route, domain::TeleopRoute::kDrop);
    EXPECT_EQ(use_case_->mode(), DriveMode::kAutomatic);
    EXPECT_EQ(output_->navigation_stops, 0);
}

TEST_F(DriveModeUseCaseTest, LeavingAutomaticByRequestStopsNavigation)
{
    make(DriveMode::kAssisted);
    use_case_->request(DriveMode::kAutomatic, allAvailable());
    use_case_->request(DriveMode::kManual, allAvailable());

    EXPECT_EQ(output_->navigation_stops, 1);
}

TEST_F(DriveModeUseCaseTest, GuardedCommandsFollowTheMode)
{
    make(DriveMode::kAssisted);
    EXPECT_TRUE(use_case_->acceptGuarded(domain::GuardedSource::kTeleop));
    EXPECT_FALSE(use_case_->acceptGuarded(domain::GuardedSource::kNavigation));

    use_case_->request(DriveMode::kAutomatic, allAvailable());
    EXPECT_FALSE(use_case_->acceptGuarded(domain::GuardedSource::kTeleop));
    EXPECT_TRUE(use_case_->acceptGuarded(domain::GuardedSource::kNavigation));
}

TEST_F(DriveModeUseCaseTest, GuardChangesArePublished)
{
    make(DriveMode::kAssisted);

    auto stopped = runningGuard();
    stopped.received = true;
    stopped.action = domain::CollisionAction::kStop;
    use_case_->onGuardReports(stopped, runningGuard());

    EXPECT_EQ(output_->published.back().guard, GuardState::kStopped);

    const auto published_before = output_->published.size();
    use_case_->onGuardReports(stopped, runningGuard());
    EXPECT_EQ(output_->published.size(), published_before) << "unchanged state republished";
}

TEST_F(DriveModeUseCaseTest, LostMissionManagerForcesAFallbackAfterTheGrace)
{
    make(DriveMode::kAssisted);
    use_case_->request(DriveMode::kAutomatic, allAvailable());

    auto gone = allAvailable();
    gone.mission_manager_available = false;

    use_case_->tick(gone, 100.0);
    use_case_->tick(gone, 101.5);
    EXPECT_EQ(use_case_->mode(), DriveMode::kAutomatic);

    use_case_->tick(gone, 102.5);
    EXPECT_EQ(use_case_->mode(), DriveMode::kAssisted);
    EXPECT_EQ(use_case_->reason(), "mission manager lost");
    EXPECT_EQ(output_->navigation_stops, 1);
}

TEST_F(DriveModeUseCaseTest, MissionManagerBlipDoesNotAccumulate)
{
    make(DriveMode::kAssisted);
    use_case_->request(DriveMode::kAutomatic, allAvailable());

    auto gone = allAvailable();
    gone.mission_manager_available = false;

    use_case_->tick(gone, 100.0);
    use_case_->tick(allAvailable(), 101.0);
    use_case_->tick(gone, 102.0);
    use_case_->tick(gone, 103.5);

    EXPECT_EQ(use_case_->mode(), DriveMode::kAutomatic);
}

}  // namespace
}  // namespace rover_drive_mode::application
