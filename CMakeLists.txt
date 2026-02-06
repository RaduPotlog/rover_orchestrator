cmake_minimum_required(VERSION 3.8)
project(rover_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    behaviortree_cpp_v3
    navigation2
    nav2_bringup
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
    nav_msgs
    nav2_msgs
    tf2_geometry_msgs
    yaml-cpp
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()