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

#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "rover_mission_manager/application/run_mission_use_case.hpp"

using rover_mission_manager::application::RunMissionUseCase;
using rover_mission_manager::domain::Mission;
using rover_mission_manager::domain::MissionPolicy;
using rover_mission_manager::domain::MissionState;
using rover_mission_manager::domain::RoverConditions;
using rover_mission_manager::domain::Waypoint;
using rover_mission_manager::domain::ports::DispatchResult;
using rover_mission_manager::domain::ports::MissionStatusPublisherPort;
using rover_mission_manager::domain::ports::NavigationPort;
using rover_mission_manager::domain::ports::NavigationResult;

namespace
{

class FakeNavigation : public NavigationPort
{
public:
    DispatchResult dispatch = DispatchResult::kDispatched;
    NavigationResult next_result = NavigationResult::kIdle;
    std::vector<Waypoint> dispatched;
    int cancels = 0;

    DispatchResult goTo(const Waypoint & waypoint) override
    {
        if (dispatch != DispatchResult::kDispatched) {
            return dispatch;
        }

        dispatched.push_back(waypoint);
        next_result = NavigationResult::kPending;
        return DispatchResult::kDispatched;
    }

    void cancel() override
    {
        ++cancels;
        next_result = NavigationResult::kIdle;
    }

    NavigationResult result() const override { return next_result; }
};

class RecordingStatusPublisher : public MissionStatusPublisherPort
{
public:
    std::vector<MissionState> states;

    void publish(const Mission & mission) override { states.push_back(mission.state()); }
};

RoverConditions unlocked()
{
    RoverConditions conditions;
    conditions.motion_locked = false;
    conditions.battery_fraction = 0.9;
    return conditions;
}

RoverConditions locked()
{
    RoverConditions conditions = unlocked();
    conditions.motion_locked = true;
    return conditions;
}

struct Fixture
{
    std::shared_ptr<FakeNavigation> navigation = std::make_shared<FakeNavigation>();
    std::shared_ptr<RecordingStatusPublisher> status =
        std::make_shared<RecordingStatusPublisher>();
    RunMissionUseCase use_case{navigation, status, MissionPolicy(0.10)};

    void acceptTwoWaypointMission()
    {
        use_case.accept(Mission("m1", {Waypoint{1.0, 0.0, 0.0}, Waypoint{2.0, 0.0, 0.0}}));
    }
};

}  // namespace

TEST(RunMissionUseCaseTest, RejectsNullPorts)
{
    auto navigation = std::make_shared<FakeNavigation>();
    auto status = std::make_shared<RecordingStatusPublisher>();

    EXPECT_THROW(
        RunMissionUseCase(nullptr, status, MissionPolicy(0.1)), std::invalid_argument);
    EXPECT_THROW(
        RunMissionUseCase(navigation, nullptr, MissionPolicy(0.1)), std::invalid_argument);
}

TEST(RunMissionUseCaseTest, DrivesWaypointsInOrderUntilTheMissionSucceeds)
{
    Fixture f;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());
    ASSERT_EQ(f.navigation->dispatched.size(), 1u);
    EXPECT_DOUBLE_EQ(f.navigation->dispatched[0].x, 1.0);

    f.navigation->next_result = NavigationResult::kReached;
    f.use_case.tick(unlocked());
    ASSERT_EQ(f.navigation->dispatched.size(), 2u);
    EXPECT_DOUBLE_EQ(f.navigation->dispatched[1].x, 2.0);

    f.navigation->next_result = NavigationResult::kReached;
    f.use_case.tick(unlocked());
    EXPECT_EQ(f.use_case.mission().state(), MissionState::kSucceeded);
}

TEST(RunMissionUseCaseTest, DoesNotRedispatchWhileAGoalIsInFlight)
{
    Fixture f;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());
    f.use_case.tick(unlocked());
    f.use_case.tick(unlocked());

    EXPECT_EQ(f.navigation->dispatched.size(), 1u);
}

TEST(RunMissionUseCaseTest, HoldsAndCancelsTheGoalWhileLocked)
{
    Fixture f;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());
    ASSERT_EQ(f.navigation->dispatched.size(), 1u);

    f.use_case.tick(locked());

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kHeldByLock);
    EXPECT_EQ(f.navigation->cancels, 2);  // one on accept(), one on hold
}

TEST(RunMissionUseCaseTest, ResumesTheSameWaypointAfterAHold)
{
    Fixture f;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());
    f.use_case.tick(locked());
    f.use_case.tick(unlocked());

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kRunning);
    EXPECT_EQ(f.use_case.mission().currentIndex(), 0u);

    ASSERT_EQ(f.navigation->dispatched.size(), 2u);
    EXPECT_DOUBLE_EQ(f.navigation->dispatched[1].x, 1.0);
}

TEST(RunMissionUseCaseTest, FailsWhenTheNavigatorIsUnreachable)
{
    Fixture f;
    f.navigation->dispatch = DispatchResult::kUnreachable;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kFailed);
    EXPECT_EQ(f.use_case.mission().failureReason(), "navigator unreachable");
}

// The adapter reports kNotReady while Nav 2 is still coming up, instead of blocking the manager's
// timer in wait_for_action_server(). The mission must keep running and dispatch once it is up.
TEST(RunMissionUseCaseTest, RetriesWhileTheNavigatorIsNotReady)
{
    Fixture f;
    f.navigation->dispatch = DispatchResult::kNotReady;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());
    f.use_case.tick(unlocked());

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kRunning);
    EXPECT_TRUE(f.navigation->dispatched.empty());

    f.navigation->dispatch = DispatchResult::kDispatched;
    f.use_case.tick(unlocked());

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kRunning);
    ASSERT_EQ(f.navigation->dispatched.size(), 1u);
    EXPECT_DOUBLE_EQ(f.navigation->dispatched[0].x, 1.0);
}

TEST(RunMissionUseCaseTest, FailsWhenNavigationReportsFailure)
{
    Fixture f;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());
    f.navigation->next_result = NavigationResult::kFailed;
    f.use_case.tick(unlocked());

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kFailed);
    EXPECT_EQ(f.use_case.mission().failureReason(), "navigation failed");
}

TEST(RunMissionUseCaseTest, AbortsOnAFlatBattery)
{
    Fixture f;
    f.acceptTwoWaypointMission();
    f.use_case.tick(unlocked());

    RoverConditions flat = unlocked();
    flat.battery_fraction = 0.01;
    f.use_case.tick(flat);

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kFailed);
    EXPECT_EQ(f.use_case.mission().failureReason(), "aborted by mission policy");
}

TEST(RunMissionUseCaseTest, TickingATerminalMissionIsAHarmlessNoOp)
{
    Fixture f;
    f.acceptTwoWaypointMission();
    f.use_case.cancel();

    const auto dispatched_before = f.navigation->dispatched.size();
    f.use_case.tick(unlocked());
    f.use_case.tick(unlocked());

    EXPECT_EQ(f.use_case.mission().state(), MissionState::kCancelled);
    EXPECT_EQ(f.navigation->dispatched.size(), dispatched_before);
}

TEST(RunMissionUseCaseTest, PublishesOnTransitionsRatherThanEveryTick)
{
    Fixture f;
    f.acceptTwoWaypointMission();

    f.use_case.tick(unlocked());
    f.use_case.tick(unlocked());
    f.use_case.tick(unlocked());

    // accept() -> RUNNING is the only transition so far; the idle ticks add nothing.
    EXPECT_EQ(f.status->states.size(), 1u);
    EXPECT_EQ(f.status->states.front(), MissionState::kRunning);

    f.navigation->next_result = NavigationResult::kReached;
    f.use_case.tick(unlocked());

    // Still RUNNING, but the waypoint cursor moved, which operators need to see.
    EXPECT_EQ(f.status->states.size(), 2u);
}
