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

#include "rover_navigation/plugins/condition/is_lidar_healthy.hpp"

#include <memory>
#include <string>

namespace rover_navigation
{

IsLidarHealthy::IsLidarHealthy(const std::string & condition_name, const BT::NodeConfig & conf)
: BT::ConditionNode(condition_name, conf),
  last_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  status_received_(false),
  last_msg_time_ns_(0),
  topic_("diagnostics"),
  status_name_("rover_rs16_lidar_node: Lidar status"),
  timeout_(3.0),
  require_present_(false)
{
  getInput("topic", topic_);
  getInput("status_name", status_name_);
  getInput("timeout", timeout_);
  getInput("require_present", require_present_);

  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");

  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread_ = std::thread([this]() { callback_group_executor_.spin(); });

  // Depth 20, not the KeepLast(1) IsMotionLocked uses: /diagnostics is shared by every node
  // on the rover (battery, drivers, GPS, LEDs, safety), so a depth-1 queue would routinely
  // drop the one message this node cares about. diagnostic_updater publishes reliably.
  //
  // nav2::LifecycleNode::create_subscription takes (topic, callback, qos, callback_group) -
  // note the argument order differs from rclcpp's, and the group replaces SubscriptionOptions.
  diagnostics_sub_ = node_->create_subscription<DiagnosticArrayMsg>(
    topic_,
    std::bind(&IsLidarHealthy::diagnosticsCb, this, std::placeholders::_1),
    rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    callback_group_);
}

IsLidarHealthy::~IsLidarHealthy()
{
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

void IsLidarHealthy::diagnosticsCb(const DiagnosticArrayMsg::SharedPtr msg)
{
  for (const auto & status : msg->status) {
    if (status.name != status_name_) {
      continue;
    }

    last_level_ = static_cast<unsigned char>(status.level);
    last_msg_time_ns_ = node_->now().nanoseconds();
    status_received_ = true;
    return;
  }
}

bool IsLidarHealthy::isHealthy() const
{
  if (!status_received_) {
    // Never seen. Either no lidar is fitted, or rover_bringup's 10 s TimerAction has not
    // released it yet. Treated as "not applicable" unless the operator asked otherwise.
    return !require_present_;
  }

  if (timeout_ > 0.0) {
    const auto age_ns = node_->now().nanoseconds() - last_msg_time_ns_.load();
    if (age_ns > static_cast<rcl_time_point_value_t>(timeout_ * 1e9)) {
      return false;
    }
  }

  // OK and WARN are both usable: rover_rs16_lidar raises WARN for a sparse cloud or a rate below
  // min_rate_ratio, which degrades the costmap but does not invalidate it. ERROR is a cloud
  // timeout and STALE is "no data yet" -- neither may be driven on.
  const auto level = last_level_.load();
  return level == diagnostic_msgs::msg::DiagnosticStatus::OK ||
         level == diagnostic_msgs::msg::DiagnosticStatus::WARN;
}

BT::NodeStatus IsLidarHealthy::tick()
{
  if (isHealthy()) {
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Lidar unhealthy: '%s' on '%s' reports level %u (or is stale). Halting navigation.",
    status_name_.c_str(), topic_.c_str(), static_cast<unsigned>(last_level_.load()));

  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_navigation

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rover_navigation::IsLidarHealthy>("IsLidarHealthy");
}
