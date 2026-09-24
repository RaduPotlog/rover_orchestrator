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

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/mission_state.hpp>
#include <rover_msgs/srv/set_mission.hpp>

#include "rover_mission_manager/infrastructure/mission_manager_node.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using rover_mission_manager::infrastructure::MissionManagerNode;
using rover_msgs::msg::MissionState;

namespace
{

class MissionManagerInitializeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        const auto options = rclcpp::NodeOptions().arguments({
            "--ros-args",
            "--params-file", ROVER_MISSION_MANAGER_CONFIG,
            "-p", std::string("bt_project_path:=") + ROVER_MISSION_MANAGER_TREE,
        });
        node_ = std::make_shared<MissionManagerNode>("mission_manager", options);

        client_node_ = std::make_shared<rclcpp::Node>("mission_manager_test_client");
        set_mission_ = client_node_->create_client<rover_msgs::srv::SetMission>("set_mission");
        state_sub_ = client_node_->create_subscription<MissionState>(
            "mission_state", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            [this](const MissionState::SharedPtr msg) {
                seen_states_.push_back(msg->state);
                if (waiting_for_ && msg->state == *waiting_for_) {
                    waiting_for_.reset();
                    state_seen_.set_value();
                }
            });

        executor_.add_node(node_->get_node_base_interface());
        executor_.add_node(client_node_);
    }

    void TearDown() override
    {
        executor_.remove_node(client_node_);
        executor_.remove_node(node_->get_node_base_interface());
    }

    /** @brief Spin until mission_state has reported @p state, or @p timeout. */
    bool waitForState(uint8_t state, std::chrono::milliseconds timeout = 5s)
    {
        // It may already have arrived while spinning for something else.
        if (std::find(seen_states_.begin(), seen_states_.end(), state) != seen_states_.end()) {
            return true;
        }

        state_seen_ = std::promise<void>();
        waiting_for_ = state;
        return executor_.spin_until_future_complete(state_seen_.get_future(), timeout) ==
               rclcpp::FutureReturnCode::SUCCESS;
    }

    std::shared_ptr<MissionManagerNode> node_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Client<rover_msgs::srv::SetMission>::SharedPtr set_mission_;
    rclcpp::Subscription<MissionState>::SharedPtr state_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    std::vector<uint8_t> seen_states_;
    std::optional<uint8_t> waiting_for_;
    std::promise<void> state_seen_;
};

}  // namespace

// Builds the real node from the shipped config/mission_manager.yaml and behavior_trees/
// rover_mission.xml, exactly as the launch file does. createTree() only fails at startup - a
// node type in the tree missing from plugin_libs, a blackboard entry of the wrong type, a leaf
// that cannot read "node" - so this is where a drift between the two files shows up.
TEST_F(MissionManagerInitializeTest, BuildsTheShippedTreeAndCyclesThroughItsLifecycle)
{
    ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);

    // A mission with no motion_lock publisher reads as locked, so once the tick timer runs
    // the use case moves it RUNNING -> HELD. Seeing HELD proves the node ticks.
    ASSERT_TRUE(set_mission_->wait_for_service(5s));
    auto request = std::make_shared<rover_msgs::srv::SetMission::Request>();
    request->waypoints.resize(1);
    request->waypoints[0].pose.orientation.w = 1.0;
    auto response = set_mission_->async_send_request(request);
    ASSERT_EQ(
        executor_.spin_until_future_complete(response, 5s), rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(response.get()->success) << response.get()->message;
    EXPECT_TRUE(waitForState(MissionState::HELD));

    // Deactivating pauses the manager: the mission in flight is cancelled and reported.
    ASSERT_EQ(node_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(waitForState(MissionState::CANCELLED));

    // Cleanup must release everything configure built - the factory's plugin registrations
    // and the Groot2 port included - or configuring a second time throws.
    ASSERT_EQ(node_->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
    ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(node_->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(MissionManagerInitializeTest, RefusesMissionsWhileInactive)
{
    ASSERT_EQ(node_->configure().id(), State::PRIMARY_STATE_INACTIVE);

    ASSERT_TRUE(set_mission_->wait_for_service(5s));
    auto request = std::make_shared<rover_msgs::srv::SetMission::Request>();
    request->waypoints.resize(1);
    request->waypoints[0].pose.orientation.w = 1.0;
    auto response = set_mission_->async_send_request(request);
    ASSERT_EQ(
        executor_.spin_until_future_complete(response, 5s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(response.get()->success);
}
