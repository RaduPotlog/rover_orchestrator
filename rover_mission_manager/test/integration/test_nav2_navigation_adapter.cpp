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

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rover_mission_manager/infrastructure/nav2_navigation_adapter.hpp"

using namespace std::chrono_literals;
using rover_mission_manager::domain::Waypoint;
using rover_mission_manager::domain::ports::NavigationResult;
using rover_mission_manager::infrastructure::Nav2NavigationAdapter;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

namespace
{

/** @brief Stand-in for bt_navigator: accepts every goal and finishes it only when told to. */
class FakeNavigator
{
public:
    explicit FakeNavigator(const rclcpp::Node::SharedPtr & node)
    {
        server_ = rclcpp_action::create_server<NavigateToPose>(
            node, "navigate_to_pose",
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<ServerGoalHandle>) {
                ++cancel_requests;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<ServerGoalHandle> handle) { goals.push_back(handle); });
    }

    std::vector<std::shared_ptr<ServerGoalHandle>> goals;
    int cancel_requests = 0;

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
};

class Nav2NavigationAdapterTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        server_node_ = std::make_shared<rclcpp::Node>("fake_bt_navigator");
        client_node_ = std::make_shared<rclcpp::Node>("mission_manager_under_test");
        navigator_ = std::make_unique<FakeNavigator>(server_node_);
        adapter_ = std::make_unique<Nav2NavigationAdapter>(
            client_node_.get(), "navigate_to_pose", "map", 5s);
        executor_.add_node(server_node_);
        executor_.add_node(client_node_);
    }

    void TearDown() override
    {
        executor_.remove_node(client_node_);
        executor_.remove_node(server_node_);
    }

    /** @brief Spin until the predicate holds; false on timeout. */
    bool spinUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 5s)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (predicate()) {
                return true;
            }
            executor_.spin_some(10ms);
        }
        return predicate();
    }

    /**
     * @brief Spin for a fixed window, for asserting that something does NOT happen.
     *
     * The window is several times what the positive-control tests need for a result to
     * arrive, so a result that was going to be applied would have been.
     */
    void spinFor(std::chrono::milliseconds window)
    {
        spinUntil([] { return false; }, window);
    }

    void startGoal()
    {
        const auto before = navigator_->goals.size();
        ASSERT_TRUE(adapter_->goTo(Waypoint{1.0, 2.0, 0.5}));
        ASSERT_TRUE(spinUntil([&] { return navigator_->goals.size() == before + 1; }));
    }

    static std::shared_ptr<NavigateToPose::Result> emptyResult()
    {
        return std::make_shared<NavigateToPose::Result>();
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Node::SharedPtr server_node_;
    rclcpp::Node::SharedPtr client_node_;
    std::unique_ptr<FakeNavigator> navigator_;
    std::unique_ptr<Nav2NavigationAdapter> adapter_;
};

}  // namespace

TEST_F(Nav2NavigationAdapterTest, ReportsTheOutcomeOfTheGoalInFlight)
{
    startGoal();
    EXPECT_EQ(adapter_->result(), NavigationResult::kPending);
    navigator_->goals.back()->succeed(emptyResult());
    EXPECT_TRUE(spinUntil([&] { return adapter_->result() == NavigationResult::kReached; }));

    startGoal();
    navigator_->goals.back()->abort(emptyResult());
    EXPECT_TRUE(spinUntil([&] { return adapter_->result() == NavigationResult::kFailed; }));
}

TEST_F(Nav2NavigationAdapterTest, IgnoresALateAbortFromAGoalItCancelled)
{
    // The GoTo-during-motion-lock race: the manager holds and cancels, and bt_navigator's own
    // IsMotionLocked check aborts the same goal. That ABORTED used to turn the hold into
    // "navigation failed" as soon as the lock cleared.
    startGoal();
    // Let the goal response reach the client, so cancel() has a handle and this exercises the
    // late-result path; a cancel that beats the response is the next test's case.
    spinFor(300ms);
    adapter_->cancel();
    ASSERT_TRUE(spinUntil([&] { return navigator_->cancel_requests == 1; }));

    navigator_->goals.back()->abort(emptyResult());
    spinFor(500ms);

    EXPECT_EQ(adapter_->result(), NavigationResult::kIdle);
}

TEST_F(Nav2NavigationAdapterTest, CancelsAGoalWhoseResponseArrivesAfterTheCancel)
{
    // A hold on the tick right after dispatch: there is no goal handle to cancel yet.
    ASSERT_TRUE(adapter_->goTo(Waypoint{1.0, 2.0, 0.5}));
    adapter_->cancel();

    EXPECT_TRUE(spinUntil([&] { return navigator_->cancel_requests == 1; }))
        << "the goal was accepted after cancel() and never cancelled";

    ASSERT_EQ(navigator_->goals.size(), 1u);
    navigator_->goals.back()->canceled(emptyResult());
    spinFor(500ms);
    EXPECT_EQ(adapter_->result(), NavigationResult::kIdle);
}

TEST_F(Nav2NavigationAdapterTest, IgnoresTheResultOfAReplacedGoal)
{
    startGoal();
    startGoal();

    navigator_->goals.front()->abort(emptyResult());
    spinFor(500ms);
    EXPECT_EQ(adapter_->result(), NavigationResult::kPending);

    navigator_->goals.back()->succeed(emptyResult());
    EXPECT_TRUE(spinUntil([&] { return adapter_->result() == NavigationResult::kReached; }));
}

TEST_F(Nav2NavigationAdapterTest, TreatsSomeoneElsesCancelAsAFailure)
{
    // Idle here would make the use case re-dispatch the waypoint the operator just stopped.
    startGoal();

    auto other_client =
        rclcpp_action::create_client<NavigateToPose>(server_node_, "navigate_to_pose");
    other_client->async_cancel_all_goals();
    ASSERT_TRUE(spinUntil([&] { return navigator_->cancel_requests == 1; }));

    navigator_->goals.back()->canceled(emptyResult());
    EXPECT_TRUE(spinUntil([&] { return adapter_->result() == NavigationResult::kFailed; }));
}
