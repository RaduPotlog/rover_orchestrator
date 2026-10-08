cmake_minimum_required(VERSION 3.10)
project(rover_follow_me)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/follow_me_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Rules and the executor against a fake action: no ROS graph, safe to run in parallel.
  ament_add_pytest_test(test_follow_rules test/unit/test_follow_rules.py)
  ament_add_pytest_test(test_follow_executor test/unit/test_follow_executor.py)
  add_test(
    NAME check_domain_purity
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
