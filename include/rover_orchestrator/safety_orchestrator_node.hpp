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

namespace rover_orchestrator
{

class SafetyOrchestratorNode : public rclcpp::Node
{

public:

    SafetyOrchestratorNode(
        const std::string & node_name, 
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node(node_name, options)
    {
        RCLCPP_INFO(this->get_logger(), "Constructing node.");
    }

    ~SafetyOrchestratorNode() 
    {

    }

    void init() 
    {

    }

private:

};

}  // namespace rover_orchestrator

#endif  // ROVER_ORCHESTRATOR_SAFETY_ORCHESTRATOR_HPP_