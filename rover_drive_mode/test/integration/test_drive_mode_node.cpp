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

// ROS-level tests for DriveModeNode: that the routing the use-case tests describe reaches the
// topics the platform consumes. Topic names and QoS live only in the node, and a mistake in
// either yields a mode that silently routes nothing - or routes around the guard.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/drive_mode.hpp>
#include <rover_msgs/srv/set_drive_mode.hpp>
#include <rover_msgs/srv/set_mission.hpp>

#include "rover_drive_mode/infrastructure/drive_mode_node.hpp"

namespace rover_drive_mode::infrastructure
{
namespace
{

using namespace std::chrono_literals;
using geometry_msgs::msg::TwistStamped;
using nav2_msgs::msg::CollisionMonitorState;
using rover_msgs::msg::DriveMode;

rclcpp::QoS commandQos() { return rclcpp::QoS(rclcpp::KeepLast(10)).reliable(); }

TwistStamped twist(double linear, double angular)
{
    TwistStamped msg;
    msg.header.stamp.sec = 42;  // passed through untouched, checked below
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    return msg;
}

class DriveModeNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        harness_ = std::make_shared<rclcpp::Node>("drive_mode_test_harness");

        web_pub_ = harness_->create_publisher<TwistStamped>("teleop_web_cmd_vel_stamped", commandQos());
        guard_out_pub_ = harness_->create_publisher<TwistStamped>("teleop_guard_out", commandQos());
        nav_guarded_pub_ =
            harness_->create_publisher<TwistStamped>("nav_cmd_vel_guarded", commandQos());

        record(teleop_output_, "teleop_driver_interface_cmd_vel_stamped");
        record(guard_in_, "teleop_guard_in");
        record(nav_output_, "nav_cmd_vel_stamped");

        mode_client_ = harness_->create_client<rover_msgs::srv::SetDriveMode>("set_drive_mode");

        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("timer_frequency", 50.0),
            rclcpp::Parameter("mission_manager_loss_grace", 0.3),
        });
        node_ = std::make_shared<DriveModeNode>("drive_mode_manager", options);

        executor_.add_node(harness_);
        executor_.add_node(node_);
    }

    void TearDown() override
    {
        executor_.remove_node(node_);
        executor_.remove_node(harness_);
        node_.reset();
        mission_service_.reset();
        guard_state_pub_.reset();
        harness_.reset();
    }

    void record(std::vector<TwistStamped> & sink, const std::string & topic)
    {
        subs_.push_back(harness_->create_subscription<TwistStamped>(
            topic, commandQos(), [&sink](const TwistStamped & msg) { sink.push_back(msg); }));
    }

    template <typename PredicateT>
    bool spinUntil(PredicateT predicate, const std::chrono::milliseconds timeout = 3000ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();

            if (predicate()) {
                return true;
            }

            std::this_thread::sleep_for(2ms);
        }

        executor_.spin_some();
        return predicate();
    }

    // Spins for a fixed window to show that something does NOT arrive.
    void spinFor(const std::chrono::milliseconds window)
    {
        spinUntil([] { return false; }, window);
    }

    // A subscriber that joins after the node published: only a latched publisher reaches it.
    std::optional<DriveMode> latestMode()
    {
        std::optional<DriveMode> latest;
        auto sub = harness_->create_subscription<DriveMode>(
            "drive_mode", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [&latest](const DriveMode & msg) { latest = msg; });
        spinUntil([&latest] { return latest.has_value(); }, 2000ms);
        return latest;
    }

    bool waitForMode(std::uint8_t mode, std::uint8_t guard = 255)
    {
        std::optional<DriveMode> latest;
        auto sub = harness_->create_subscription<DriveMode>(
            "drive_mode", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [&latest](const DriveMode & msg) { latest = msg; });
        return spinUntil([&] {
            return latest.has_value() && latest->mode == mode &&
                   (guard == 255 || latest->guard == guard);
        });
    }

    rover_msgs::srv::SetDriveMode::Response::SharedPtr requestMode(std::uint8_t mode)
    {
        EXPECT_TRUE(spinUntil([this] { return mode_client_->service_is_ready(); }));
        auto request = std::make_shared<rover_msgs::srv::SetDriveMode::Request>();
        request->mode = mode;
        auto future = mode_client_->async_send_request(request);
        EXPECT_TRUE(spinUntil([&future] {
            return future.wait_for(0s) == std::future_status::ready;
        }));
        return future.get();
    }

    void startMissionManager()
    {
        mission_service_ = harness_->create_service<rover_msgs::srv::SetMission>(
            "set_mission",
            [](
                const std::shared_ptr<rover_msgs::srv::SetMission::Request>,
                std::shared_ptr<rover_msgs::srv::SetMission::Response> response) {
                response->success = true;
            });
    }

    std::shared_ptr<rclcpp::Node> harness_;
    std::shared_ptr<DriveModeNode> node_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Publisher<TwistStamped>::SharedPtr web_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr guard_out_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr nav_guarded_pub_;
    rclcpp::Publisher<CollisionMonitorState>::SharedPtr guard_state_pub_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
    rclcpp::Client<rover_msgs::srv::SetDriveMode>::SharedPtr mode_client_;
    rclcpp::Service<rover_msgs::srv::SetMission>::SharedPtr mission_service_;

    std::vector<TwistStamped> teleop_output_;
    std::vector<TwistStamped> guard_in_;
    std::vector<TwistStamped> nav_output_;
};

TEST_F(DriveModeNodeTest, BootsInAssistedLatchedForLateJoiners)
{
    const auto mode = latestMode();
    ASSERT_TRUE(mode.has_value());
    EXPECT_EQ(mode->mode, DriveMode::ASSISTED);
    EXPECT_EQ(mode->reason, "boot default");
    // No teleop collision monitor on the graph.
    EXPECT_EQ(mode->guard, DriveMode::GUARD_NO_DATA);
}

TEST_F(DriveModeNodeTest, AssistedRoutesTheJoystickThroughTheGuard)
{
    ASSERT_TRUE(spinUntil([this] { return web_pub_->get_subscription_count() > 0; }));

    web_pub_->publish(twist(0.5, 0.1));
    ASSERT_TRUE(spinUntil([this] { return !guard_in_.empty(); }));
    EXPECT_TRUE(teleop_output_.empty()) << "ASSISTED command bypassed the guard";
    EXPECT_EQ(guard_in_.front().header.stamp.sec, 42);

    // What the monitor lets through goes on to the platform, header intact.
    guard_out_pub_->publish(twist(0.2, 0.1));
    ASSERT_TRUE(spinUntil([this] { return !teleop_output_.empty(); }));
    EXPECT_DOUBLE_EQ(teleop_output_.front().twist.linear.x, 0.2);
    EXPECT_EQ(teleop_output_.front().header.stamp.sec, 42);
}

TEST_F(DriveModeNodeTest, ManualRoutesTheJoystickStraightToThePlatform)
{
    const auto response = requestMode(DriveMode::MANUAL);
    ASSERT_TRUE(response->success) << response->message;
    EXPECT_EQ(response->mode, DriveMode::MANUAL);
    ASSERT_TRUE(waitForMode(DriveMode::MANUAL, DriveMode::GUARD_BYPASSED));

    web_pub_->publish(twist(0.5, 0.0));
    ASSERT_TRUE(spinUntil([this] { return !teleop_output_.empty(); }));
    EXPECT_TRUE(guard_in_.empty());

    // A late command from the guard must not reach the platform in MANUAL.
    teleop_output_.clear();
    guard_out_pub_->publish(twist(0.3, 0.0));
    spinFor(200ms);
    EXPECT_TRUE(teleop_output_.empty());
}

TEST_F(DriveModeNodeTest, AutomaticIsRefusedWithoutTheMissionManager)
{
    const auto response = requestMode(DriveMode::AUTOMATIC);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->mode, DriveMode::ASSISTED);
    EXPECT_NE(response->message.find("mission manager"), std::string::npos);
}

TEST_F(DriveModeNodeTest, UnknownModeIsRefused)
{
    const auto response = requestMode(0);
    EXPECT_FALSE(response->success);
    EXPECT_EQ(response->mode, DriveMode::ASSISTED);
}

TEST_F(DriveModeNodeTest, NavigationReachesThePlatformOnlyInAutomatic)
{
    ASSERT_TRUE(spinUntil([this] { return nav_guarded_pub_->get_subscription_count() > 0; }));

    nav_guarded_pub_->publish(twist(0.6, 0.0));
    spinFor(200ms);
    EXPECT_TRUE(nav_output_.empty()) << "Nav 2 drove the rover in ASSISTED";

    startMissionManager();
    const auto response = requestMode(DriveMode::AUTOMATIC);
    ASSERT_TRUE(response->success) << response->message;

    nav_guarded_pub_->publish(twist(0.6, 0.0));
    ASSERT_TRUE(spinUntil([this] { return !nav_output_.empty(); }));
    EXPECT_DOUBLE_EQ(nav_output_.back().twist.linear.x, 0.6);
}

TEST_F(DriveModeNodeTest, MovingTheStickInAutomaticTakesOver)
{
    startMissionManager();
    ASSERT_TRUE(requestMode(DriveMode::AUTOMATIC)->success);

    // A centred stick (the UI's release burst) is dropped and does not take over.
    web_pub_->publish(twist(0.0, 0.0));
    spinFor(200ms);
    EXPECT_TRUE(guard_in_.empty());
    EXPECT_TRUE(teleop_output_.empty());

    web_pub_->publish(twist(0.4, 0.0));
    ASSERT_TRUE(spinUntil([this] { return !guard_in_.empty(); }));
    ASSERT_TRUE(waitForMode(DriveMode::ASSISTED));
    EXPECT_EQ(latestMode()->reason, "operator takeover");

    // Leaving AUTOMATIC sends a zero on the Nav 2 input so the mux does not hold its last command.
    ASSERT_TRUE(spinUntil([this] { return !nav_output_.empty(); }));
    EXPECT_DOUBLE_EQ(nav_output_.back().twist.linear.x, 0.0);
}

TEST_F(DriveModeNodeTest, LosingTheMissionManagerLeavesAutomatic)
{
    startMissionManager();
    ASSERT_TRUE(requestMode(DriveMode::AUTOMATIC)->success);

    mission_service_.reset();
    ASSERT_TRUE(waitForMode(DriveMode::ASSISTED));
    EXPECT_EQ(latestMode()->reason, "mission manager lost");
}

TEST_F(DriveModeNodeTest, GuardStateFollowsTheTeleopCollisionMonitor)
{
    ASSERT_TRUE(waitForMode(DriveMode::ASSISTED, DriveMode::GUARD_NO_DATA));

    guard_state_pub_ = harness_->create_publisher<CollisionMonitorState>(
        "teleop_collision_monitor_state", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    ASSERT_TRUE(waitForMode(DriveMode::ASSISTED, DriveMode::GUARD_CLEAR));

    CollisionMonitorState state;
    state.action_type = CollisionMonitorState::STOP;
    state.polygon_name = "teleop_stop";
    ASSERT_TRUE(spinUntil([this] { return guard_state_pub_->get_subscription_count() > 0; }));
    guard_state_pub_->publish(state);
    ASSERT_TRUE(waitForMode(DriveMode::ASSISTED, DriveMode::GUARD_STOPPED));

    state.polygon_name = "invalid source";
    guard_state_pub_->publish(state);
    ASSERT_TRUE(waitForMode(DriveMode::ASSISTED, DriveMode::GUARD_NO_DATA));

    // The monitor goes away: its last report must not linger.
    guard_state_pub_.reset();
    ASSERT_TRUE(waitForMode(DriveMode::ASSISTED, DriveMode::GUARD_NO_DATA));
}

}  // namespace
}  // namespace rover_drive_mode::infrastructure
