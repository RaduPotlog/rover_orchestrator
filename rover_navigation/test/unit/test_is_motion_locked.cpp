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
//
// Tests for the IsMotionLocked BT condition, which had none.
//
// It is the rover's analogue of an E-Stop condition inside a navigation tree, and its whole
// contract is a fail-safe default: an engaged lock, a lock that never arrived, and a lock that
// went stale must all read the same way. Those are precisely the paths a unit test has to pin,
// because in the field they look like "navigation carried on" rather than like a failure.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rover_navigation/plugins/condition/is_motion_locked.hpp"

namespace rover_navigation
{
namespace
{

using namespace std::chrono_literals;

class IsMotionLockedTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        node_ = std::make_shared<nav2::LifecycleNode>("is_motion_locked_test_node");

        publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
            "motion_lock", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        factory_.registerNodeType<IsMotionLocked>("IsMotionLocked");

        config_ = BT::NodeConfig{};
        config_.blackboard = BT::Blackboard::create();
        config_.blackboard->set("node", node_);
    }

    void TearDown() override
    {
        condition_.reset();
        publisher_.reset();
        node_.reset();
    }

    void makeCondition(const double timeout_s)
    {
        config_.input_ports["topic"] = "motion_lock";
        config_.input_ports["timeout"] = std::to_string(timeout_s);

        condition_ = std::make_unique<IsMotionLocked>("is_motion_locked", config_);
    }

    void publish(const bool locked)
    {
        std_msgs::msg::Bool msg;
        msg.data = locked;
        publisher_->publish(msg);
    }

    // The condition spins its own callback group on its own thread, so the test only has to wait
    // for the tick to reflect a published value rather than drive an executor itself.
    bool tickUntil(const BT::NodeStatus expected, const std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            if (condition_->tick() == expected) {
                return true;
            }

            std::this_thread::sleep_for(5ms);
        }

        return condition_->tick() == expected;
    }

    nav2::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    BT::BehaviorTreeFactory factory_;
    BT::NodeConfig config_;
    std::unique_ptr<IsMotionLocked> condition_;
};

// Before anything has been published there is no evidence the rover may move, and the node must
// not supply the benefit of the doubt.
TEST_F(IsMotionLockedTest, ReportsLockedBeforeAnyMessageArrives)
{
    makeCondition(0.5);

    EXPECT_EQ(condition_->tick(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsMotionLockedTest, ReportsUnlockedWhileAFreshUnlockedMessageIsAvailable)
{
    makeCondition(0.5);

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    bool unlocked = false;

    while (std::chrono::steady_clock::now() < deadline && !unlocked) {
        publish(false);
        unlocked = tickUntil(BT::NodeStatus::FAILURE, 100ms);
    }

    EXPECT_TRUE(unlocked) << "a fresh unlocked message never produced FAILURE";
}

TEST_F(IsMotionLockedTest, ReportsLockedWhileALockedMessageIsAvailable)
{
    makeCondition(0.5);

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    bool locked = false;

    while (std::chrono::steady_clock::now() < deadline && !locked) {
        publish(true);
        locked = tickUntil(BT::NodeStatus::SUCCESS, 100ms);
    }

    EXPECT_TRUE(locked);
}

// The case that matters most: rover_motion_lock_node republishes at 10 Hz precisely so that a
// dead publisher closes the mux rather than opening it. This node must not disagree, or a tree
// would keep navigating against a mux that has already stopped passing commands.
TEST_F(IsMotionLockedTest, ReLocksOnceTheLastMessageGoesStale)
{
    makeCondition(0.2);

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    bool unlocked = false;

    while (std::chrono::steady_clock::now() < deadline && !unlocked) {
        publish(false);
        unlocked = tickUntil(BT::NodeStatus::FAILURE, 100ms);
    }

    ASSERT_TRUE(unlocked) << "precondition: the condition should have reported unlocked first";

    // Stop publishing; the 0.2 s timeout must take over.
    EXPECT_TRUE(tickUntil(BT::NodeStatus::SUCCESS, 3000ms))
        << "the condition never re-locked after the publisher went silent";
}

// timeout 0.0 documents "disable the staleness check", so an unlocked message must stay trusted.
// Worth pinning because it is the one configuration in which this node is NOT fail-safe, and that
// should be a deliberate choice rather than something that drifts in.
TEST_F(IsMotionLockedTest, ZeroTimeoutDisablesTheStalenessCheck)
{
    makeCondition(0.0);

    const auto deadline = std::chrono::steady_clock::now() + 3s;
    bool unlocked = false;

    while (std::chrono::steady_clock::now() < deadline && !unlocked) {
        publish(false);
        unlocked = tickUntil(BT::NodeStatus::FAILURE, 100ms);
    }

    ASSERT_TRUE(unlocked);

    std::this_thread::sleep_for(400ms);

    EXPECT_EQ(condition_->tick(), BT::NodeStatus::FAILURE)
        << "a zero timeout should not age the last message out";
}

}  // namespace
}  // namespace rover_navigation
