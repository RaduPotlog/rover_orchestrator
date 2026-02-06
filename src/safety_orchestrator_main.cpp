


#include "rover_orchestrator/safety_orchestrator_node.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto safety_orchestrator_node =
        std::make_shared<rover_orchestrator::SafetyOrchestratorNode>("safety_orchestrator");
    
    safety_orchestrator_node->init();

    try {
        rclcpp::spin(safety_orchestrator_node);
    } catch (const std::runtime_error & err) {
        std::cerr << "[safety_orchestrator] Caught exception: " << err.what() << std::endl;
    }

    std::cout << "[safety_orchestrator] Shutting down" << std::endl;
    
    rclcpp::shutdown();
    
    return 0;
}