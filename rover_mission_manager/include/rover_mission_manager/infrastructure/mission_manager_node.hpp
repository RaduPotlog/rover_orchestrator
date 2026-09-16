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

#ifndef ROVER_MISSION_MANAGER_INFRASTRUCTURE_MISSION_MANAGER_NODE_HPP_
#define ROVER_MISSION_MANAGER_INFRASTRUCTURE_MISSION_MANAGER_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "rover_mission_manager/application/run_mission_use_case.hpp"
#include "rover_mission_manager/infrastructure/behavior_tree_runner.hpp"
#include "rover_mission_manager/mission_manager_parameters.hpp"

namespace rover_mission_manager::infrastructure
{

/**
 * @brief Ticks the mission behavior tree and keeps its blackboard fed from ROS topics.
 *
 * The same shape as rover_safety's safety node and husarion's lights/safety/docking managers:
 * a wall timer drives one `tickOnce()`, BT leaf plugins named by parameters do the ROS work,
 * and a Groot2 publisher exposes the tree for live inspection.
 *
 * Two layers run here on purpose. The tree is the *operator-facing* policy -- what a mission
 * is made of, and which leaf nodes run in what order -- and is meant to be edited in Groot2
 * without recompiling. RunMissionUseCase is the *invariant* part: waypoint bookkeeping and
 * the safety rules that must hold whatever the tree says.
 */
class MissionManagerNode : public rclcpp::Node
{
public:
    explicit MissionManagerNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Build the tree, wire the adapters and start ticking.
     * @throws std::runtime_error when the BehaviorTree project cannot be loaded.
     *
     * Separate from the constructor because registering ROS BT plugins needs
     * shared_from_this(), which is not available until construction has finished.
     */
    void initialize();

private:
    void registerBehaviorTree();
    void timerCb();

    void motionLockCb(const std_msgs::msg::Bool::SharedPtr msg);
    void batteryCb(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void diagnosticsCb(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

    void runMissionCb(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /** @brief Sample the conditions the mission policy decides on. */
    domain::RoverConditions currentConditions() const;

    /** @brief `<namespace>/odom` when goal_frame_id is left empty. */
    std::string resolveGoalFrameId() const;

    std::shared_ptr<mission_manager::ParamListener> param_listener_;
    mission_manager::Params params_;

    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BehaviorTreeRunner> mission_tree_runner_;
    std::unique_ptr<application::RunMissionUseCase> run_mission_use_case_;

    rclcpp::TimerBase::SharedPtr mission_tree_timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_lock_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_mission_srv_;

    std::atomic<bool> motion_locked_;
    std::atomic<bool> motion_lock_received_;
    std::atomic<rcl_time_point_value_t> motion_lock_stamp_ns_;
    std::atomic<double> battery_fraction_;
    std::atomic<unsigned char> lidar_level_;
    std::atomic<bool> lidar_status_received_;
    std::atomic<rcl_time_point_value_t> lidar_stamp_ns_;
};

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_MISSION_MANAGER_NODE_HPP_
