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
  add_test(
    NAME check_domain_purity
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
