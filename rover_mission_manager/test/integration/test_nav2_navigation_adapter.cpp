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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rover_mission_manager/infrastructure/nav2_navigation_adapter.hpp"

using namespace std::chrono_literals;
using nav2_msgs::action::NavigateToPose;
using rover_mission_manager::domain::Waypoint;
using rover_mission_manager::domain::ports::DispatchResult;
using rover_mission_manager::domain::ports::NavigationResult;
using rover_mission_manager::infrastructure::Nav2NavigationAdapter;

namespace
{

using ServerGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

/** @brief A navigate_to_pose server that accepts every goal and finishes only when told. */
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
                std::lock_guard<std::mutex> lock(mutex_);
                ++cancel_requests_;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<ServerGoalHandle> handle) {
                std::lock_guard<std::mutex> lock(mutex_);
                goal_ = handle;
            });
    }

    int cancelRequests() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_requests_;
    }

    std::shared_ptr<ServerGoalHandle> goal() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return goal_;
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
    mutable std::mutex mutex_;
    int cancel_requests_ = 0;
    std::shared_ptr<ServerGoalHandle> goal_;
};

class Nav2NavigationAdapterTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        host_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("adapter_host");
        server_node_ = std::make_shared<rclcpp::Node>("fake_navigator");
        navigator_ = std::make_unique<FakeNavigator>(server_node_);
        adapter_ = std::make_unique<Nav2NavigationAdapter>(
            host_.get(), "navigate_to_pose", "odom", std::chrono::duration<double>(5.0));

        executor_.add_node(host_->get_node_base_interface());
        executor_.add_node(server_node_);
    }

    void TearDown() override
    {
        adapter_.reset();
        executor_.remove_node(server_node_);
        executor_.remove_node(host_->get_node_base_interface());
    }

    /** @brief Spin until @p done holds or @p timeout passes; returns whether it held. */
    bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout = 5s)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done()) {
            if (std::chrono::steady_clock::now() > deadline) {
                return false;
            }
            executor_.spin_some(10ms);
        }
        return true;
    }

    /**
     * @brief Keep spinning for @p duration. Only for waits with no event to observe: "nothing
     * further happens" checks, and letting the goal response reach the adapter (which exposes
     * no hook for it). Everything else waits on a condition with spinUntil(). A window that
     * turns out too short under load does not fail those tests; it only exercises the
     * cancel-before-response path, which has its own test.
     */
    void spinFor(std::chrono::milliseconds duration)
    {
        spinUntil([] { return false; }, duration);
    }

    /** @brief goTo() once the server is discoverable. Does not spin after dispatching. */
    void dispatch()
    {
        DispatchResult dispatched = DispatchResult::kNotReady;
        ASSERT_TRUE(spinUntil([&] {
            dispatched = adapter_->goTo(Waypoint{1.0, 2.0, 0.0});
            return dispatched != DispatchResult::kNotReady;
        }));
        ASSERT_EQ(dispatched, DispatchResult::kDispatched);
    }

    /** @brief Cancel the server's goal the way another client (RViz, the CLI) would. */
    void cancelFromAnotherClient()
    {
        auto other = rclcpp_action::create_client<NavigateToPose>(server_node_, "navigate_to_pose");
        auto cancelled = other->async_cancel_all_goals();
        ASSERT_EQ(
            executor_.spin_until_future_complete(cancelled, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
    }

    rclcpp_lifecycle::LifecycleNode::SharedPtr host_;
    rclcpp::Node::SharedPtr server_node_;
    std::unique_ptr<FakeNavigator> navigator_;
    std::unique_ptr<Nav2NavigationAdapter> adapter_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

}  // namespace

TEST_F(Nav2NavigationAdapterTest, SucceededGoalIsReached)
{
    dispatch();
    EXPECT_EQ(adapter_->result(), NavigationResult::kPending);

    ASSERT_TRUE(spinUntil([&] { return navigator_->goal() != nullptr; }));
    navigator_->goal()->succeed(std::make_shared<NavigateToPose::Result>());

    EXPECT_TRUE(spinUntil([&] { return adapter_->result() == NavigationResult::kReached; }));
}

// The race the adapter exists to close: a hold or abort decided on the tick right after
// goTo(), before the server's goal response has arrived. The goal the server accepts must
// still be cancelled, or Nav 2 drives to it while the mission believes it has stopped.
TEST_F(Nav2NavigationAdapterTest, CancelBeforeTheGoalResponseStillCancelsTheGoal)
{
    dispatch();
    adapter_->cancel();
    EXPECT_EQ(adapter_->result(), NavigationResult::kIdle);

    ASSERT_TRUE(spinUntil([&] { return navigator_->cancelRequests() == 1; }));

    // The server confirms the cancel; that stale CANCELED must not surface as a failure.
    ASSERT_TRUE(spinUntil([&] { return navigator_->goal() != nullptr; }));
    ASSERT_TRUE(spinUntil([&] { return navigator_->goal()->is_canceling(); }));
    navigator_->goal()->canceled(std::make_shared<NavigateToPose::Result>());
    spinFor(300ms);
    EXPECT_EQ(adapter_->result(), NavigationResult::kIdle);
}

TEST_F(Nav2NavigationAdapterTest, CancelAfterTheGoalResponseCancelsTheGoal)
{
    dispatch();
    ASSERT_TRUE(spinUntil([&] { return navigator_->goal() != nullptr; }));
    // Let the goal response reach the adapter too, so it holds the handle. No observable
    // event for this; see spinFor().
    spinFor(200ms);

    adapter_->cancel();
    EXPECT_TRUE(spinUntil([&] { return navigator_->cancelRequests() == 1; }));
    EXPECT_EQ(adapter_->result(), NavigationResult::kIdle);
}

// Another client cancelling our goal is not a pause: reporting kIdle would make the use case
// send the same waypoint straight back and override the operator.
TEST_F(Nav2NavigationAdapterTest, CancelByAnotherClientFailsTheGoal)
{
    dispatch();
    ASSERT_TRUE(spinUntil([&] { return navigator_->goal() != nullptr; }));
    spinFor(200ms);

    cancelFromAnotherClient();
    ASSERT_TRUE(navigator_->goal()->is_canceling());
    navigator_->goal()->canceled(std::make_shared<NavigateToPose::Result>());

    EXPECT_TRUE(spinUntil([&] { return adapter_->result() == NavigationResult::kFailed; }));
}
