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

#ifndef ROVER_MISSION_MANAGER_INFRASTRUCTURE_BEHAVIOR_TREE_RUNNER_HPP_
#define ROVER_MISSION_MANAGER_INFRASTRUCTURE_BEHAVIOR_TREE_RUNNER_HPP_

#include <any>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>

#include <rover_utils/networking_utils.hpp>

namespace rover_mission_manager::infrastructure
{

/**
 * @brief Owns one BehaviorTree: its blackboard, its Groot2 publisher and its tick.
 *
 * Generalised from rover_safety::BehaviorTreeSafety (rover_ros). It is duplicated rather than
 * reused because rover_safety ships in a separate repository, its class is safety-specific,
 * and its CMakeLists exports `include/rover_safety` rather than `include`, so its headers do
 * not resolve from a downstream package. If that class is ever promoted to rover_utils, this
 * header should be deleted in favour of it.
 */
class BehaviorTreeRunner
{
public:
    /**
     * @param tree_name   Tree ID inside the BehaviorTree project.
     * @param initial_blackboard Entries seeded before the first tick.
     * @param groot_port  Preferred Groot2 port; the next free one is used if it is taken.
     */
    BehaviorTreeRunner(
        std::string tree_name,
        std::map<std::string, std::any> initial_blackboard,
        unsigned groot_port = 1667)
    : tree_name_(std::move(tree_name)),
      initial_blackboard_(std::move(initial_blackboard)),
      groot_port_(groot_port),
      tree_status_(BT::NodeStatus::IDLE)
    {
    }

    ~BehaviorTreeRunner() = default;

    /**
     * @brief Build the tree from @p factory and start the Groot2 publisher.
     *
     * @param seed_blackboard Optional hook run on the fresh blackboard before the tree is
     * built. The initial-blackboard map only carries scalars and strings; entries with
     * richer types -- notably the `node` entry that BT leaf plugins read to get the ROS node
     * handle -- have to be set here, and they must exist before createTree() runs because
     * leaf constructors read them.
     */
    void initialize(
        BT::BehaviorTreeFactory & factory,
        const std::function<void(BT::Blackboard::Ptr)> & seed_blackboard = {})
    {
        config_ = createConfig(initial_blackboard_);

        if (seed_blackboard) {
            seed_blackboard(config_.blackboard);
        }

        tree_ = factory.createTree(tree_name_, config_.blackboard);

        constexpr unsigned kMaxPort = 65535;

        while (!rover_utils::isPortAvailable(groot_port_)) {
            if (groot_port_ >= kMaxPort) {
                throw std::runtime_error("No available port for the Groot2 publisher.");
            }

            RCLCPP_WARN_STREAM(
                logger(), "Port " << groot_port_ << " is not available. Trying the next one.");

            ++groot_port_;
        }

        RCLCPP_INFO_STREAM(logger(), "Groot2 publisher started on port " << groot_port_ << ".");

        groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot_port_);
    }

    void tickOnce() { tree_status_ = tree_.tickOnce(); }
    void haltTree() { tree_.haltTree(); }

    BT::NodeStatus treeStatus() const { return tree_status_; }
    BT::Tree & tree() { return tree_; }
    BT::Blackboard::Ptr blackboard() const { return config_.blackboard; }
    unsigned grootPort() const { return groot_port_; }

private:
    static rclcpp::Logger logger() { return rclcpp::get_logger("BehaviorTreeRunner"); }

    /**
     * @brief Seed a blackboard from a type-erased map.
     * @throws std::invalid_argument on an unsupported entry type.
     */
    static BT::NodeConfig createConfig(const std::map<std::string, std::any> & values)
    {
        BT::NodeConfig config;
        config.blackboard = BT::Blackboard::create();

        for (const auto & [name, value] : values) {
            const std::type_info & type = value.type();

            if (type == typeid(bool)) {
                config.blackboard->set<bool>(name, std::any_cast<bool>(value));
            } else if (type == typeid(int)) {
                config.blackboard->set<int>(name, std::any_cast<int>(value));
            } else if (type == typeid(unsigned)) {
                config.blackboard->set<unsigned>(name, std::any_cast<unsigned>(value));
            } else if (type == typeid(float)) {
                config.blackboard->set<float>(name, std::any_cast<float>(value));
            } else if (type == typeid(double)) {
                config.blackboard->set<double>(name, std::any_cast<double>(value));
            } else if (type == typeid(const char *)) {
                config.blackboard->set<std::string>(name, std::any_cast<const char *>(value));
            } else if (type == typeid(std::string)) {
                config.blackboard->set<std::string>(name, std::any_cast<std::string>(value));
            } else {
                throw std::invalid_argument(
                    "Invalid type for blackboard entry '" + name +
                    "'. Valid types are: bool, int, unsigned, float, double, const char*, "
                    "std::string.");
            }
        }

        return config;
    }

    std::string tree_name_;
    std::map<std::string, std::any> initial_blackboard_;
    unsigned groot_port_;

    BT::Tree tree_;
    BT::NodeStatus tree_status_;
    BT::NodeConfig config_;
    std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
};

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_BEHAVIOR_TREE_RUNNER_HPP_
