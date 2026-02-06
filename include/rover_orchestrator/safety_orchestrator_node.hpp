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

#ifndef ROVER_ORCHESTRATOR_SAFETY_ORCHESTRATOR_HPP_
#define ROVER_ORCHESTRATOR_SAFETY_ORCHESTRATOR_HPP_

#include <map>
#include <memory>
#include <string>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include <std_srvs/srv/set_bool.hpp>
#include "sensor_msgs/msg/battery_state.hpp"

#include "rover_msgs/msg/gpio_state.hpp"
#include "rover_msgs/msg/rover_driver_state.hpp"
#include "rover_msgs/msg/system_status.hpp"

#include "rover_orchestrator/behavior_tree_orchestrator.hpp"
#include "rover_orchestrator/safety_orchestrator_parameters.hpp"

namespace rover_orchestrator
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using RoverDriverStateMsg = rover_msgs::msg::RoverDriverState;
using IOStateMsg = rover_msgs::msg::GpioState;
using SystemStatusMsg = rover_msgs::msg::SystemStatus;

class SafetyOrchestratorNode : public rclcpp::Node
{

public:

    SafetyOrchestratorNode(
        const std::string & node_name, 
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    ~SafetyOrchestratorNode();

    void init();

protected:

    void declareParameters();
    
    void registerBehaviorTree();
    
    std::map<std::string, std::any> createSafetyInitialBlackboard();

    bool systemReady();

    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BehaviorTreeOrchestrator> safety_tree_orchestrator_;

    std::shared_ptr<safety_orchestrator::ParamListener> param_listener_;
    safety_orchestrator::Params params_;

private:
    
    static constexpr float kCriticalBatteryTemp = 50.0;
    static constexpr float kFatalBatteryTemp = 60.0;
    static constexpr char kShutdownLocalhostCommand[] =
        "dbus-send --system --print-reply --dest=org.freedesktop.login1 /org/freedesktop/login1 "
        "org.freedesktop.login1.Manager.PowerOff boolean:true";

    void batterySubscriberCallback(const BatteryStateMsg::SharedPtr battery);
    void systemStatusSubscriberCallback(const SystemStatusMsg::SharedPtr system_status);
    void safetyTreeTimerCallback();

    rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
    rclcpp::Subscription<RoverDriverStateMsg>::SharedPtr driver_state_sub_;
    rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
    rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
    rclcpp::Subscription<SystemStatusMsg>::SharedPtr system_status_sub_;
    rclcpp::TimerBase::SharedPtr safety_tree_timer_;
    
    double battery_temp_{0.0};
    double cpu_temp_{0.0};
};

}  // namespace rover_orchestrator

#endif  // ROVER_ORCHESTRATOR_SAFETY_ORCHESTRATOR_HPP_