cmake_minimum_required(VERSION 3.8)
project(rosplane_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  Eigen3
)
ament_export_include_directories(include)

include_directories(  #use this if you need .h files for include statements.  The include will need to have the directories where each .h is respectively.
        include
        ${EIGEN3_INCLUDE_DIRS}
        )


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


add_executable(rosplane_gazebo_truth_publisher
        src/gazebo_state_transcription.cpp)
ament_target_dependencies(rosplane_gazebo_truth_publisher rosplane_msgs rclcpp rclpy nav_msgs)
install(TARGETS
        rosplane_gazebo_truth_publisher
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
