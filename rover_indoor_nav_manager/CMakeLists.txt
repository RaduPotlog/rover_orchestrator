cmake_minimum_required(VERSION 3.10)
project(rover_indoor_nav_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/indoor_nav_manager_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Domain + application with fakes, plus the file repository and the launch command line.
  # None of them need a ROS graph, so they are safe to run in parallel with anything.
  ament_add_pytest_test(test_domain test/unit/test_domain.py)
  ament_add_pytest_test(test_indoor_nav_service test/unit/test_indoor_nav_service.py)
  ament_add_pytest_test(test_file_map_repository test/unit/test_file_map_repository.py)
  ament_add_pytest_test(test_launch_controller test/unit/test_launch_controller.py)
  ament_add_pytest_test(test_motion test/unit/test_motion.py)
  ament_add_pytest_test(test_initial_pose_msg test/unit/test_initial_pose_msg.py)
  ament_add_pytest_test(test_initial_pose_seeder test/unit/test_initial_pose_seeder.py)

  # The node itself over a real rclpy graph (services, worker thread, latched QoS). Pinned to
  # FastDDS on localhost when it is installed: the workspace's rmw_zenoh needs a router, and
  # the login shell points Zenoh at the rover's, which off the rover hangs rclpy.init and on
  # it joins the live graph.
  find_package(rmw_fastrtps_cpp QUIET)
  if(rmw_fastrtps_cpp_FOUND)
    set(ISOLATED_ROS_TEST_ENV
      RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  endif()
  ament_add_pytest_test(test_indoor_nav_node test/integration/test_indoor_nav_node.py
    TIMEOUT 120
    ENV ${ISOLATED_ROS_TEST_ENV})

  add_test(
    NAME check_domain_purity
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
