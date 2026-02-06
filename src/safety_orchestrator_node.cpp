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

#include "rover_orchestrator/safety_orchestrator_node.hpp"

#include <algorithm>
#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rover_orchestrator
{

SafetyOrchestratorNode::SafetyOrchestratorNode(
    const std::string & node_name, 
    const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
    RCLCPP_INFO(this->get_logger(), "Constructing node.");

    this->param_listener_ =
        std::make_shared<safety_manager::ParamListener>(this->get_node_parameters_interface());
    
    this->params_ = this->param_listener_->get_params();

    const auto safety_initial_blackboard = createSafetyInitialBlackboard();
    safety_tree_orchestrator_ = std::make_unique<BehaviorTreeOrchestrator>(
        "SafetyOrchestrator", safety_initial_blackboard, 6666);
    
    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

SafetyOrchestratorNode::~SafetyOrchestratorNode()
{

}

void SafetyOrchestratorNode::init()
{
    RCLCPP_INFO(this->get_logger(), "Initializing.");

    registerBehaviorTree();
    safety_tree_orchestrator_->init(factory_);

    battery_sub_ = this->create_subscription<BatteryStateMsg>(
        "rover_battery/battery_status", 10, 
        std::bind(&SafetyOrchestratorNode::batterySubscriberCallback, this, std::placeholders::_1));
  
    system_status_sub_ = this->create_subscription<SystemStatusMsg>(
        "system_status", 10, 
        std::bind(&SafetyOrchestratorNode::systemStatusSubscriberCallback, this, std::placeholders::_1));

    const double timer_freq = this->params_.timer_frequency;
    const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

    safety_tree_timer_ = this->create_wall_timer(
        timer_period, std::bind(&SafetyOrchestratorNode::safetyTreeTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void SafetyOrchestratorNode::registerBehaviorTree()
{
    const auto bt_project_path = this->params_.bt_project_path;

    const auto plugin_libs = this->params_.plugin_libs;
    const auto ros_plugin_libs = this->params_.ros_plugin_libs;

    // const auto service_availability_timeout = this->params_.ros_communication_timeout.availability;
    // const auto service_response_timeout = this->params_.ros_communication_timeout.response;

    // BT::RosNodeParams params;
    
    // params.nh = this->shared_from_this();
    // auto wait_for_server_timeout_s = std::chrono::duration<double>(service_availability_timeout);
    // params.wait_for_server_timeout =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(wait_for_server_timeout_s);
    // auto server_timeout_s = std::chrono::duration<double>(service_response_timeout);
    // params.server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(server_timeout_s);

    // behavior_tree_utils::registerBehaviorTree(
    //     factory_, bt_project_path, plugin_libs, params, ros_plugin_libs);

    RCLCPP_INFO_STREAM(this->get_logger(), "BehaviorTree registered from path '" << bt_project_path << "'");
}

std::map<std::string, std::any> SafetyOrchestratorNode::createSafetyInitialBlackboard()
{
    const std::map<std::string, std::any> safety_initial_bb = {
        {"CRITICAL_BAT_TEMP", kCriticalBatteryTemp},
        {"FATAL_BAT_TEMP", kFatalBatteryTemp},
        // battery health constants
        {"POWER_SUPPLY_HEALTH_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN)},
        {"POWER_SUPPLY_HEALTH_GOOD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD)},
        {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
        {"POWER_SUPPLY_HEALTH_DEAD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD)},
        {"POWER_SUPPLY_HEALTH_OVERVOLTAGE", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE)},
        {"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE",
        unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)},
        {"POWER_SUPPLY_HEALTH_COLD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD)},
        {"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE",
        unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE)},
        {"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE",
        unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE)},
    };

    RCLCPP_INFO(this->get_logger(), "Blackboard created.");

    return safety_initial_bb;
}

bool SafetyOrchestratorNode::systemReady()
{
    if (!safety_tree_orchestrator_->getBlackboard()->getAny("e_stop_state") ||
        !safety_tree_orchestrator_->getBlackboard()->getAny("battery_status") ||
        !safety_tree_orchestrator_->getBlackboard()->getAny("cpu_temp") ||
        !safety_tree_orchestrator_->getBlackboard()->getAny("driver_temp")) {

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, "Waiting for required system messages to arrive.");
    
        return false;
    }

    return true;
}

void SafetyOrchestratorNode::batterySubscriberCallback(const BatteryStateMsg::SharedPtr battery)
{
    const auto battery_status = battery->power_supply_status;
    const auto battery_health = battery->power_supply_health;
    
    safety_tree_orchestrator_->getBlackboard()->set<unsigned>("battery_status", battery_status);
    safety_tree_orchestrator_->getBlackboard()->set<unsigned>("battery_health", battery_health);

    if (battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
        battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
        battery_temp_ = battery->temperature;
    }

    safety_tree_orchestrator_->getBlackboard()->set<double>("bat_temp", battery_temp_);
}

void SafetyOrchestratorNode::systemStatusSubscriberCallback(const SystemStatusMsg::SharedPtr system_status)
{
    cpu_temp_ = system_status->cpu_temp;
    safety_tree_orchestrator_->getBlackboard()->set<double>("cpu_temp", cpu_temp_);
}

void SafetyOrchestratorNode::safetyTreeTimerCallback()
{
    if (!systemReady()) {
        return;
    }

    safety_tree_orchestrator_->tickOnce();

    if (safety_tree_orchestrator_->getTreeStatus() == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(this->get_logger(), "Safety behavior tree returned FAILURE status");
    }

    std::pair<bool, std::string> signal_shutdown;
    
    if (safety_tree_orchestrator_->getBlackboard()->get<std::pair<bool, std::string>>(
        "signal_shutdown", signal_shutdown)) {

        if (signal_shutdown.first) {
            // ShutdownRobot(signal_shutdown.second);
        }
    }
}

}  // namespace rover_orchestrator