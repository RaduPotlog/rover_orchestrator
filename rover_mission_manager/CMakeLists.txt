cmake_minimum_required(VERSION 3.10)
project(rover_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
    ament_cmake
    behaviortree_cpp
    diagnostic_msgs
    generate_parameter_library
    geometry_msgs
    lifecycle_msgs
    nav2_msgs
    nav2_ros_common
    rclcpp
    rclcpp_action
    rover_msgs
    rover_utils
    sensor_msgs
    std_msgs
    std_srvs)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

generate_parameter_library(mission_manager_parameters src/mission_manager_parameters.yaml)

# Domain + application: plain C++, no ROS dependencies. Kept in its own target so the
# boundary is enforced by the linker as well as by scripts/check_domain_purity.sh.
add_library(${PROJECT_NAME}_core STATIC
    src/domain/mission.cpp
    src/domain/mission_policy.cpp
    src/application/run_mission_use_case.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
set_property(TARGET ${PROJECT_NAME}_core PROPERTY POSITION_INDEPENDENT_CODE ON)

# Infrastructure: ROS 2 and BehaviorTree adapters.
add_library(${PROJECT_NAME}_ros STATIC
    src/infrastructure/mission_manager_node.cpp
    src/infrastructure/mission_request.cpp
    src/infrastructure/nav2_navigation_adapter.cpp
    src/infrastructure/ros_mission_status_publisher.cpp)
# rover_utils is header-only and exports no CMake targets, so its include directory has to
# be added by hand (same as rover_safety does).
target_include_directories(${PROJECT_NAME}_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rover_utils_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros
    ${PROJECT_NAME}_core
    mission_manager_parameters
    ${behaviortree_cpp_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${nav2_ros_common_TARGETS}
    ${rclcpp_TARGETS}
    ${rclcpp_action_TARGETS}
    ${rover_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS})
set_property(TARGET ${PROJECT_NAME}_ros PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(mission_manager_node src/rover_mission_manager.cpp)
target_link_libraries(mission_manager_node ${PROJECT_NAME}_ros)

install(TARGETS mission_manager_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_mission test/unit/test_mission.cpp)
    target_link_libraries(test_mission ${PROJECT_NAME}_core)

    ament_add_gtest(test_mission_policy test/unit/test_mission_policy.cpp)
    target_link_libraries(test_mission_policy ${PROJECT_NAME}_core)

    ament_add_gtest(test_run_mission_use_case test/unit/test_run_mission_use_case.cpp)
    target_link_libraries(test_run_mission_use_case ${PROJECT_NAME}_core)

    ament_add_gtest(test_mission_request test/unit/test_mission_request.cpp)
    target_link_libraries(test_mission_request ${PROJECT_NAME}_ros)

    # Builds the node from the shipped config and tree. is_motion_locked_bt_node comes from
    # rover_navigation's install/lib, which colcon puts on the library path for its dependents.
    ament_add_gtest(test_mission_manager_initialize
                    test/integration/test_mission_manager_initialize.cpp
                    TIMEOUT 60)
    target_compile_definitions(test_mission_manager_initialize PRIVATE
        ROVER_MISSION_MANAGER_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/mission_manager.yaml"
        ROVER_MISSION_MANAGER_TREE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/rover_mission.xml")
    target_link_libraries(test_mission_manager_initialize ${PROJECT_NAME}_ros)

    # Mechanically enforce that domain/ never grows a ROS or BehaviorTree dependency.
    add_test(
        NAME check_domain_purity
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
