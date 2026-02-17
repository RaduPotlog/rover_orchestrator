cmake_minimum_required(VERSION 3.8)
project(rover_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
set(DEPENDENCIES
    ament_cmake
    lifecycle_msgs
    nav_msgs
    nav2_msgs
    rclcpp
    std_msgs)

include_directories(include)

foreach(PACKAGE IN ITEMS ${DEPENDENCIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

# Rest of the files
install(DIRECTORY config launch map
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION include)

ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)

ament_package()
