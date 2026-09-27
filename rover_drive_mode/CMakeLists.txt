cmake_minimum_required(VERSION 3.10)
project(rover_drive_mode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
    ament_cmake
    diagnostic_msgs
    diagnostic_updater
    generate_parameter_library
    geometry_msgs
    nav2_msgs
    rclcpp
    rover_msgs)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

generate_parameter_library(drive_mode_parameters src/drive_mode_parameters.yaml)

# Domain + application: plain C++, no ROS dependencies. Kept in its own target so the
# boundary is enforced by the linker as well as by scripts/check_domain_purity.sh.
add_library(${PROJECT_NAME}_core STATIC
    src/domain/command_router.cpp
    src/domain/drive_mode.cpp
    src/domain/drive_mode_policy.cpp
    src/domain/guard_state.cpp
    src/application/drive_mode_use_case.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
set_property(TARGET ${PROJECT_NAME}_core PROPERTY POSITION_INDEPENDENT_CODE ON)

# Infrastructure: the ROS 2 node and its output adapter.
add_library(${PROJECT_NAME}_ros STATIC
    src/infrastructure/drive_mode_node.cpp
    src/infrastructure/ros_drive_mode_output.cpp)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_ros
    ${PROJECT_NAME}_core
    drive_mode_parameters
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${rover_msgs_TARGETS})
set_property(TARGET ${PROJECT_NAME}_ros PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(drive_mode_manager src/drive_mode_manager.cpp)
target_link_libraries(drive_mode_manager ${PROJECT_NAME}_ros)

install(TARGETS drive_mode_manager DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)

    ament_add_gtest(test_command_router test/unit/test_command_router.cpp)
    target_link_libraries(test_command_router ${PROJECT_NAME}_core)

    ament_add_gtest(test_drive_mode_policy test/unit/test_drive_mode_policy.cpp)
    target_link_libraries(test_drive_mode_policy ${PROJECT_NAME}_core)

    ament_add_gtest(test_guard_state test/unit/test_guard_state.cpp)
    target_link_libraries(test_guard_state ${PROJECT_NAME}_core)

    ament_add_gtest(test_drive_mode_use_case test/unit/test_drive_mode_use_case.cpp)
    target_link_libraries(test_drive_mode_use_case ${PROJECT_NAME}_core)

    # ROS-level: that routing, the latched mode, the takeover and the Nav 2 gate reach the
    # topics the platform consumes. The QoS and topic names live only in the node.
    ament_add_gtest(test_drive_mode_node test/integration/test_drive_mode_node.cpp TIMEOUT 120)
    target_link_libraries(test_drive_mode_node ${PROJECT_NAME}_ros ${PROJECT_NAME}_core)

    # Config: the collision monitor polygons cover every command and keep clear of the rover.
    ament_add_pytest_test(test_collision_monitor_config
        test/unit/test_collision_monitor_config.py)

    # Launch: node names, lifecycle wiring and the <namespace>/ substitution.
    ament_add_pytest_test(test_launch_structure test/launch/test_launch_structure.py)

    # Mechanically enforce that domain/ never grows a ROS dependency.
    add_test(
        NAME check_domain_purity
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
