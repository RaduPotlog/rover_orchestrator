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
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rover_mission_manager/infrastructure/mission_manager_node.hpp"

using namespace std::chrono_literals;
using rover_mission_manager::infrastructure::MissionManagerNode;

namespace
{

class MissionManagerInitializeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

}  // namespace

// Builds the real node from the shipped config/mission_manager.yaml and behavior_trees/
// rover_mission.xml, exactly as the launch file does. createTree() only fails at startup - a
// node type in the tree missing from plugin_libs, a blackboard entry of the wrong type, a leaf
// that cannot read "node" - so this is where a drift between the two files shows up.
TEST_F(MissionManagerInitializeTest, BuildsAndTicksTheShippedTree)
{
    const auto options = rclcpp::NodeOptions().arguments({
        "--ros-args",
        "--params-file", ROVER_MISSION_MANAGER_CONFIG,
        "-p", std::string("bt_project_path:=") + ROVER_MISSION_MANAGER_TREE,
    });
    auto node = std::make_shared<MissionManagerNode>("mission_manager", options);

    ASSERT_NO_THROW(node->initialize());

    // Let the 20 Hz timer tick the tree for a while; IsMotionLocked sees no message and reads
    // locked. spin_all() would return as soon as nothing is ready, before the first tick, so
    // spin on a future that never completes and let the timeout end it.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::promise<void> never;
    EXPECT_EQ(
        executor.spin_until_future_complete(never.get_future(), 300ms),
        rclcpp::FutureReturnCode::TIMEOUT);
    executor.remove_node(node);
}
