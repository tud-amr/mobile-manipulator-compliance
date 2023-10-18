cmake_minimum_required(VERSION 3.8)
project(dingo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dingo_driver_msg REQUIRED) 
find_package(Boost REQUIRED)

include_directories("../../../cpp")

add_executable(
  dingo_driver_node src/dingo_driver_node.cpp 
  ../../../cpp/dingo_driver/dingo_driver.cpp
  ../../../cpp/puma_motor_driver/driver.cpp
  ../../../cpp/puma_motor_driver/socketcan_gateway.cpp
  )
target_link_libraries(dingo_driver_node boost_thread)
ament_target_dependencies(dingo_driver_node rclcpp dingo_driver_msg)

install(TARGETS
  dingo_driver_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
