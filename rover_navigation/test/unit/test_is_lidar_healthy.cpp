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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_navigation/plugins/condition/is_lidar_healthy.hpp"

namespace rover_navigation
{
namespace
{

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

constexpr char kStatusName[] = "test_lidar: Lidar status";

class IsLidarHealthyTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        node_ = std::make_shared<nav2::LifecycleNode>("is_lidar_healthy_test_node");

        publisher_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "diagnostics", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

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

    void makeCondition(const double timeout_s, const bool require_present)
    {
        config_.input_ports["topic"] = "diagnostics";
        config_.input_ports["status_name"] = kStatusName;
        config_.input_ports["timeout"] = std::to_string(timeout_s);
        config_.input_ports["require_present"] = require_present ? "true" : "false";

        condition_ = std::make_unique<IsLidarHealthy>("is_lidar_healthy", config_);
    }

    void publish(const unsigned char level, const std::string & name = kStatusName)
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = name;
        status.level = level;

        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.status.push_back(status);
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

    /** Publishes `level` until the tick reports `expected`; discovery may drop the first ones. */
    bool publishUntil(
        const unsigned char level, const BT::NodeStatus expected,
        const std::string & name = kStatusName)
    {
        const auto deadline = std::chrono::steady_clock::now() + 3s;

        while (std::chrono::steady_clock::now() < deadline) {
            publish(level, name);

            if (tickUntil(expected, 100ms)) {
                return true;
            }
        }

        return false;
    }

    nav2::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
    BT::NodeConfig config_;
    std::unique_ptr<IsLidarHealthy> condition_;
};

// No lidar fitted, or rover_bringup has not released it yet: not a reason to stop.
TEST_F(IsLidarHealthyTest, NeverSeenIsHealthyByDefault)
{
    makeCondition(3.0, false);

    EXPECT_EQ(condition_->tick(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsLidarHealthyTest, NeverSeenIsUnhealthyWhenRequired)
{
    makeCondition(3.0, true);

    EXPECT_EQ(condition_->tick(), BT::NodeStatus::FAILURE);
}

TEST_F(IsLidarHealthyTest, OkIsHealthy)
{
    makeCondition(3.0, true);

    EXPECT_TRUE(publishUntil(DiagnosticStatus::OK, BT::NodeStatus::SUCCESS));
}

// rover_rs16_lidar raises WARN for a sparse cloud or a low rate: degraded, but still drivable.
TEST_F(IsLidarHealthyTest, WarnIsHealthy)
{
    makeCondition(3.0, true);

    EXPECT_TRUE(publishUntil(DiagnosticStatus::WARN, BT::NodeStatus::SUCCESS));
}

TEST_F(IsLidarHealthyTest, ErrorIsUnhealthy)
{
    makeCondition(3.0, false);

    EXPECT_TRUE(publishUntil(DiagnosticStatus::ERROR, BT::NodeStatus::FAILURE));
}

TEST_F(IsLidarHealthyTest, StaleLevelIsUnhealthy)
{
    makeCondition(3.0, false);

    EXPECT_TRUE(publishUntil(DiagnosticStatus::STALE, BT::NodeStatus::FAILURE));
}

// /diagnostics carries every node's statuses; only the configured name may count.
TEST_F(IsLidarHealthyTest, OtherStatusNamesAreIgnored)
{
    makeCondition(3.0, false);

    EXPECT_FALSE(publishUntil(DiagnosticStatus::ERROR, BT::NodeStatus::FAILURE, "gps: GPS status"));
    EXPECT_TRUE(publishUntil(DiagnosticStatus::ERROR, BT::NodeStatus::FAILURE));
}

TEST_F(IsLidarHealthyTest, StatusOlderThanTimeoutIsUnhealthy)
{
    makeCondition(0.3, true);
    ASSERT_TRUE(publishUntil(DiagnosticStatus::OK, BT::NodeStatus::SUCCESS));

    EXPECT_TRUE(tickUntil(BT::NodeStatus::FAILURE, 2s)) << "a stale OK never turned into FAILURE";
}

TEST_F(IsLidarHealthyTest, ZeroTimeoutDisablesTheStalenessCheck)
{
    makeCondition(0.0, true);
    ASSERT_TRUE(publishUntil(DiagnosticStatus::OK, BT::NodeStatus::SUCCESS));

    EXPECT_FALSE(tickUntil(BT::NodeStatus::FAILURE, 600ms));
}

TEST_F(IsLidarHealthyTest, UnparsableTimeoutFailsTheBuild)
{
    config_.input_ports["topic"] = "diagnostics";
    config_.input_ports["status_name"] = kStatusName;
    config_.input_ports["timeout"] = "not-a-number";
    config_.input_ports["require_present"] = "false";

    EXPECT_THROW(IsLidarHealthy("is_lidar_healthy", config_), BT::RuntimeError);
}

}  // namespace
}  // namespace rover_navigation
