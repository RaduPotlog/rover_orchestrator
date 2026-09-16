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

#ifndef ROVER_NAVIGATION_PLUGINS_CONDITION_IS_LIDAR_HEALTHY_HPP_
#define ROVER_NAVIGATION_PLUGINS_CONDITION_IS_LIDAR_HEALTHY_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/condition_node.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rover_navigation
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS while the lidar is healthy.
 *
 * rover_lidar exposes no service, no lifecycle transition and no boolean "lidar ok" topic --
 * its only health signal is a diagnostic_updater task named "Lidar status" on
 * `<namespace>/diagnostics`. This node watches that one status so the costmaps are never
 * trusted while the sensor feeding them is dead.
 *
 * Polarity is the OPPOSITE of IsMotionLocked: SUCCESS means "safe to drive", so this node
 * goes straight into the navigation trees' ReactiveSequence with no Inverter.
 *
 * Levels map as follows:
 *   - OK / WARN                     -> SUCCESS. WARN is a sparse cloud or a low rate: still
 *                                      usable data, and rover_lidar warns readily.
 *   - ERROR / STALE, or the last matching status older than `timeout` -> FAILURE.
 *   - status never seen at all      -> SUCCESS, unless `require_present` is true.
 *
 * That last rule is deliberately NOT fail-safe, and differs from IsMotionLocked. The rover
 * runs without a lidar whenever ROVER_USE_LIDAR is false, and rover_lidar sits behind a 10 s
 * TimerAction in rover_bringup even when it is true. Failing closed on "never seen" would
 * make navigation unusable in both cases. Set require_present="true" on a rover that is
 * always fitted with a lidar to get the strict behaviour.
 */
class IsLidarHealthy : public BT::ConditionNode
{
  using DiagnosticArrayMsg = diagnostic_msgs::msg::DiagnosticArray;

public:
  IsLidarHealthy(const std::string & condition_name, const BT::NodeConfig & conf);

  IsLidarHealthy() = delete;

  ~IsLidarHealthy() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "diagnostics", "diagnostic_msgs/DiagnosticArray topic to watch"),
      BT::InputPort<std::string>(
        "status_name", "rover_lidar_node: Lidar status",
        "Exact DiagnosticStatus name to match. rover_diag_manager's aggregator rewrites this "
        "to '/Rover/Lidar/Lidar status' on diagnostics_agg; the raw topic is watched instead "
        "so this does not depend on the aggregator running."),
      BT::InputPort<double>(
        "timeout", 3.0,
        "Seconds after which a missing status counts as unhealthy. diagnostic_updater "
        "publishes at 1 Hz; 0.0 disables the staleness check."),
      BT::InputPort<bool>(
        "require_present", false,
        "When true, a status that has never been seen counts as unhealthy. Leave false to "
        "keep no-lidar (ROVER_USE_LIDAR=false) operation working."),
    };
  }

private:
  void diagnosticsCb(const DiagnosticArrayMsg::SharedPtr msg);

  /** @brief True when the lidar is streaming usable data, or is legitimately absent. */
  bool isHealthy() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<DiagnosticArrayMsg>::SharedPtr diagnostics_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;

  std::atomic<unsigned char> last_level_;
  std::atomic<bool> status_received_;
  std::atomic<rcl_time_point_value_t> last_msg_time_ns_;

  std::string topic_;
  std::string status_name_;
  double timeout_;
  bool require_present_;
};

}  // namespace rover_navigation

#endif  // ROVER_NAVIGATION_PLUGINS_CONDITION_IS_LIDAR_HEALTHY_HPP_
