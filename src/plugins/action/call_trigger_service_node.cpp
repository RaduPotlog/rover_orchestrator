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

#include "rover_orchestrator/plugins/action/call_trigger_service_node.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

namespace rover_orchestrator
{

CallTriggerService::CallTriggerService(
    const std::string& name, 
    const BT::NodeConfiguration& config)
: SyncActionNode(name, config)
{
    if (!getInput<std::string>("service_name", service_name_)) {
        throw BT::RuntimeError("Missing required input [service_name]");
    }

    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList CallTriggerService::providedPorts() 
{
    return { BT::InputPort<std::string>("service_name", "/default/trigger"),
             BT::InputPort<bool>("value")};
}

BT::NodeStatus CallTriggerService::tick()
{
    if (!node_) {
        throw BT::RuntimeError("No rclcpp::Node available");
    }

    auto client = node_->create_client<std_srvs::srv::Trigger>(service_name_);

    if (!client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node_->get_logger(), "Service %s unavailable", service_name_.c_str());
        
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed");
        return BT::NodeStatus::FAILURE;
    }

    auto response = result_future.get();

    if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "Service %s succeeded", service_name_.c_str());
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(node_->get_logger(), "Service %s failed: %s", service_name_.c_str(), response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace rover_orchestrator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<rover_orchestrator::CallTriggerService>("CallTriggerService");
}