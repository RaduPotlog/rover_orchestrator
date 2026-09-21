cmake_minimum_required(VERSION 3.10)
project(rover_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
set(DEPENDENCIES
    ament_cmake
    behaviortree_cpp
    diagnostic_msgs
    lifecycle_msgs
    nav_msgs
    nav2_msgs
    nav2_ros_common
    rclcpp
    std_msgs)

include_directories(include)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: plain C++, no ROS dependencies. Kept in its own target so the
# boundary is enforced by the linker as well as by scripts/check_domain_purity.sh.
add_library(${PROJECT_NAME}_core STATIC
    src/domain/map_autosave_policy.cpp
    src/application/autosave_map_use_case.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
set_property(TARGET ${PROJECT_NAME}_core PROPERTY POSITION_INDEPENDENT_CODE ON)

# Infrastructure: ROS 2 adapters.
add_library(${PROJECT_NAME}_ros STATIC
    src/infrastructure/map_autosaver_node.cpp
    src/infrastructure/nav2_map_saver_client.cpp)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_ros
    ${PROJECT_NAME}_core
    ${nav2_msgs_TARGETS}
    ${rclcpp_TARGETS})
set_property(TARGET ${PROJECT_NAME}_ros PROPERTY POSITION_INDEPENDENT_CODE ON)

# Executables
add_executable(map_autosaver_node src/map_autosaver.cpp)
target_link_libraries(map_autosaver_node ${PROJECT_NAME}_ros)

install(TARGETS map_autosaver_node DESTINATION lib/${PROJECT_NAME})

# BehaviorTree leaf plugins, loaded by bt_navigator via `plugin_lib_names`.
add_library(is_motion_locked_bt_node SHARED src/plugins/condition/is_motion_locked.cpp)
list(APPEND plugin_libs is_motion_locked_bt_node)

add_library(is_lidar_healthy_bt_node SHARED src/plugins/condition/is_lidar_healthy.cpp)
list(APPEND plugin_libs is_lidar_healthy_bt_node)

foreach(bt_plugin ${plugin_libs})
    target_include_directories(${bt_plugin} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    target_link_libraries(${bt_plugin}
        ${behaviortree_cpp_TARGETS}
        ${diagnostic_msgs_TARGETS}
        ${nav2_ros_common_TARGETS}
        ${rclcpp_TARGETS}
        ${std_msgs_TARGETS})
    target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# bt_navigator resolves plugin_lib_names with BT::SharedLibrary, which searches the ament
# library path - so these must land in lib/, not lib/${PROJECT_NAME}.
install(
    TARGETS ${plugin_libs}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# Rest of the files
install(DIRECTORY behavior_trees config launch map
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)

    # Introspects the launch files without spawning anything, so it needs no ROS graph and
    # no nav2 binaries. Its main job is asserting that localization_source's three
    # declarations stay in sync -- they must agree by hand, and a divergence fails in the
    # field rather than at startup.
    ament_add_pytest_test(test_localization_launch test/launch/test_localization_launch.py)

    ament_add_gtest(test_map_autosave_policy test/unit/test_map_autosave_policy.cpp)
    target_link_libraries(test_map_autosave_policy ${PROJECT_NAME}_core)

    ament_add_gtest(test_autosave_map_use_case test/unit/test_autosave_map_use_case.cpp)
    target_link_libraries(test_autosave_map_use_case ${PROJECT_NAME}_core)

    # The rover's E-Stop condition inside a navigation tree. Its contract is entirely about
    # fail-safe defaults - no message and a stale message must both read as locked - and those
    # paths fail in the field as "navigation carried on" rather than as an error.
    ament_add_gtest(test_is_motion_locked test/unit/test_is_motion_locked.cpp)
    target_link_libraries(test_is_motion_locked is_motion_locked_bt_node)

    # Mechanically enforce that domain/ never grows a ROS or BehaviorTree dependency. Plain
    # CTest add_test (pass/fail via exit code) rather than ament_add_test, since the script
    # has no JUnit-XML result-file contract to satisfy.
    add_test(
        NAME check_domain_purity
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})

ament_package()
