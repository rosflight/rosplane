cmake_minimum_required(VERSION 3.8)
project(rosplane2_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ros_gz REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  ros_gz
  Eigen3
  ignition-cmake2
  ignition-msgs5
  ignition-msgs8
  ignition-common4
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME}/)

# Add the EntityWrench node
add_executable(entity_wrench_pub src/entity_wrench_pub.cpp)
ament_target_dependencies(entity_wrench_pub ignition-cmake2 ignition-msgs8 ignition-transport11 ignition-common4 rclcpp Eigen3)
install(TARGETS entity_wrench_pub DESTINATION lib/${PROJECT_NAME})

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
