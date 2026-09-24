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

#include "rover_navigation/plugins/condition/is_motion_locked.hpp"

#include <memory>
#include <string>

namespace rover_navigation
{

namespace
{

// Every port declares a default, so getInput only fails on a value that does not parse (say
// timeout="abc"). Fail the tree build then rather than run on the member default unnoticed.
template<typename T>
void readInput(const BT::TreeNode & node, const std::string & port, T & value)
{
  const auto result = node.getInput<T>(port);
  if (!result) {
    throw BT::RuntimeError(
      node.name() + ": invalid input [" + port + "]: " + result.error());
  }
  value = result.value();
}

}  // namespace

IsMotionLocked::IsMotionLocked(const std::string & condition_name, const BT::NodeConfig & conf)
: BT::ConditionNode(condition_name, conf),
  motion_locked_(true),
  message_received_(false),
  last_msg_time_ns_(0),
  topic_("motion_lock"),
  timeout_(0.5)
{
  readInput(*this, "topic", topic_);
  readInput(*this, "timeout", timeout_);

  node_ = config().blackboard->get<nav2::LifecycleNode::SharedPtr>("node");

  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread_ = std::thread([this]() { callback_group_executor_.spin(); });

  // rover_motion_lock_node publishes KeepLast(1) + reliable + VOLATILE
  // (rover_twist_mux/src/infrastructure/motion_lock_node.cpp). Requesting transient_local
  // here would make the subscription QoS-incompatible and it would silently never connect.
  //
  // nav2::LifecycleNode::create_subscription takes (topic, callback, qos, callback_group) -
  // note the argument order differs from rclcpp's, and the group replaces SubscriptionOptions.
  motion_lock_sub_ = node_->create_subscription<BoolMsg>(
    topic_,
    std::bind(&IsMotionLocked::motionLockCb, this, std::placeholders::_1),
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    callback_group_);
}

IsMotionLocked::~IsMotionLocked()
{
  callback_group_executor_.cancel();
  if (callback_group_executor_thread_.joinable()) {
    callback_group_executor_thread_.join();
  }
}

void IsMotionLocked::motionLockCb(const BoolMsg::SharedPtr msg)
{
  motion_locked_ = msg->data;
  last_msg_time_ns_ = node_->now().nanoseconds();
  message_received_ = true;
}

bool IsMotionLocked::isLocked() const
{
  if (!message_received_) {
    return true;
  }

  if (timeout_ > 0.0) {
    const auto age_ns = node_->now().nanoseconds() - last_msg_time_ns_.load();
    if (age_ns > static_cast<rcl_time_point_value_t>(timeout_ * 1e9)) {
      return true;
    }
  }

  return motion_locked_;
}

BT::NodeStatus IsMotionLocked::tick()
{
  if (isLocked()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Motion lock engaged (or '%s' is stale). Halting navigation.", topic_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rover_navigation

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rover_navigation::IsMotionLocked>("IsMotionLocked");
}
