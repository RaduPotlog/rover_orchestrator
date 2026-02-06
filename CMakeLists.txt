cmake_minimum_required(VERSION 3.8)
project(rover_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    behaviortree_cpp_v3
    generate_parameter_library
    geometry_msgs
    rover_msgs
    rover_utils
    OpenSSL
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    yaml-cpp
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

# ------------------------------------- Plugins -------------------------------------- #
add_library(call_set_bool_service_bt_node SHARED
            src/plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED
            src/plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)
# ------------------------------------- End plugins =--------------------------------- #

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_plugin} ${PACKAGE_DEPENDENCIES})
endforeach()

add_executable(safety_orchestrator_node src/safety_orchestrator_main.cpp
                                        src/safety_orchestrator_node.cpp
)

ament_target_dependencies(
    safety_orchestrator_node
    behaviortree_cpp_v3
    geometry_msgs
    rover_msgs
    rover_utils
    rclcpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
)

generate_parameter_library(safety_orchestrator_parameters
                           src/safety_orchestrator_parameters.yaml)
target_link_libraries(safety_orchestrator_node ${plugin_libs}
                      safety_orchestrator_parameters)

install(TARGETS ${plugin_libs} DESTINATION lib)

install(TARGETS safety_orchestrator_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()