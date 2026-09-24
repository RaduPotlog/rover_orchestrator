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

#ifndef ROVER_NAVIGATION_PLUGINS_CONDITION_IS_MOTION_LOCKED_HPP_
#define ROVER_NAVIGATION_PLUGINS_CONDITION_IS_MOTION_LOCKED_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/condition_node.h>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rover_navigation
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS while the rover's motion lock is engaged.
 *
 * This is the rover's analogue of an E-Stop condition. The lock is published on `motion_lock`
 * by rover_motion_lock_node (rover_twist_mux), which also gates the velocity mux at priority
 * 200. Navigation trees invert this node so that an engaged lock aborts the tree.
 *
 * Fail-safe, matching rover_twist_mux's own semantics: the lock is considered ENGAGED both
 * before the first message arrives and whenever the last message is older than `timeout`.
 * rover_motion_lock_node republishes at 10 Hz specifically so that a dead publisher closes
 * the mux rather than opening it; this node must not disagree with that.
 */
class IsMotionLocked : public BT::ConditionNode
{
  using BoolMsg = std_msgs::msg::Bool;

public:
  IsMotionLocked(const std::string & condition_name, const BT::NodeConfig & conf);

  IsMotionLocked() = delete;

  ~IsMotionLocked() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "motion_lock", "std_msgs/Bool topic carrying the motion lock state"),
      BT::InputPort<double>(
        "timeout", 0.5,
        "Seconds after which a missing motion_lock message counts as locked. The publisher "
        "runs at 10 Hz; 0.0 disables the staleness check."),
    };
  }

private:
  void motionLockCb(const BoolMsg::SharedPtr msg);

  /** @brief True when the lock is engaged, or when no fresh message is available. */
  bool isLocked() const;

  // Nav 2 1.5.x puts a nav2::LifecycleNode on the BT blackboard under "node";
  // asking for an rclcpp::Node makes BT::Any::convert throw at tree creation.
  nav2::LifecycleNode::SharedPtr node_;
  nav2::Subscription<BoolMsg>::SharedPtr motion_lock_sub_;
  // Each instance spins its own callback group on its own thread, so tick() never spins and
  // never waits. That costs one thread per instance - up to six on the rover (both navigate
  // trees plus the mission tree) - which is deliberate, not an oversight.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;

  std::atomic<bool> motion_locked_;
  std::atomic<bool> message_received_;
  std::atomic<rcl_time_point_value_t> last_msg_time_ns_;

  std::string topic_;
  double timeout_;
};

}  // namespace rover_navigation

#endif  // ROVER_NAVIGATION_PLUGINS_CONDITION_IS_MOTION_LOCKED_HPP_
