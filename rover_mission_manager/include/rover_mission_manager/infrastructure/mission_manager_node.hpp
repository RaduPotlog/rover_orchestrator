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
#include <mutex>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rover_msgs/srv/set_mission.hpp>
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
 *
 * A lifecycle node because it owns a Nav 2 goal while a mission runs:
 * - configure builds the tree, the adapters, the subscriptions and the services;
 * - activate starts the tick timer;
 * - deactivate stops it, cancels the mission (and with it the Nav 2 goal) and halts the tree,
 *   so `ros2 lifecycle set mission_manager deactivate` pauses the manager without leaving
 *   the rover driving;
 * - cleanup and shutdown release everything configure built.
 * rclcpp's pre-shutdown hook runs the shutdown transition while the context is still valid,
 * so Ctrl+C / SIGTERM cancels the goal in flight instead of abandoning it.
 */
class MissionManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit MissionManagerNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~MissionManagerNode() override;

    /** @brief The `autostart` parameter: configure and activate right after construction. */
    bool autostart() const { return params_.autostart; }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
    /**
     * @brief Build the tree and wire the adapters, subscriptions and services.
     * @throws std::runtime_error when the BehaviorTree project cannot be loaded.
     */
    void build();
    void registerBehaviorTree();

    /** @brief Stop ticking, cancel the mission and halt the tree. Safe to call twice. */
    void stopMission(const std::string & reason);
    /** @brief Drop everything build() built. Safe on a partially configured node. */
    void releaseResources();

    /** @brief rclcpp pre-shutdown hook: run the shutdown transition while ROS still works. */
    void onPreShutdown();

    bool isActive() const;

    void timerCb();

    void motionLockCb(const std_msgs::msg::Bool::SharedPtr msg);
    void batteryCb(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void diagnosticsCb(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

    void runMissionCb(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void setMissionCb(
        const std::shared_ptr<rover_msgs::srv::SetMission::Request> request,
        std::shared_ptr<rover_msgs::srv::SetMission::Response> response);

    /** @brief Sample the conditions the mission policy decides on. */
    domain::RoverConditions currentConditions() const;

    /** @brief `<namespace>/odom` when goal_frame_id is left empty. */
    std::string resolveGoalFrameId() const;

    std::shared_ptr<mission_manager::ParamListener> param_listener_;
    mission_manager::Params params_;

    // Serialises the timer, the services and the lifecycle callbacks. They all run on the
    // executor thread, except the pre-shutdown hook, which runs on rclcpp's signal thread
    // and must not tear the use case down under a tick in progress.
    std::mutex mutex_;
    std::unique_ptr<rclcpp::PreShutdownCallbackHandle> pre_shutdown_handle_;

    // Recreated on every configure: a factory rejects a second registration of the same
    // plugin, so reusing it would make configure -> cleanup -> configure throw.
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    // Handle the BT leaves get as "node". rover_navigation's conditions and nav2_behavior_tree's
    // plugins read a nav2::LifecycleNode (Nav 2 1.5), which this node is not.
    nav2::LifecycleNode::SharedPtr bt_node_;
    std::unique_ptr<BehaviorTreeRunner> mission_tree_runner_;
    std::unique_ptr<application::RunMissionUseCase> run_mission_use_case_;

    rclcpp::TimerBase::SharedPtr mission_tree_timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_lock_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_mission_srv_;
    rclcpp::Service<rover_msgs::srv::SetMission>::SharedPtr set_mission_srv_;

    std::string goal_frame_id_;
    std::size_t missions_accepted_ = 0;

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
