cmake_minimum_required(VERSION 3.8)
project(rosplane)

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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rosflight_msgs REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

ament_export_dependencies(rclcpp rclpy)
ament_export_include_directories(include)

#install(DIRECTORY originalFiles DESTINATION share/${PROJECT_NAME}/)
include_directories(  #use this if you need .h files for include statements.  The include will need to have the directories where each .h is respectively.
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)

### START OF REAL EXECUTABLES ###

# Controller
add_executable(rosplane_controller
  src/controller_base.cpp
  src/controller_state_machine.cpp
  src/controller_successive_loop.cpp
  src/controller_total_energy.cpp
  src/param_manager.cpp)
ament_target_dependencies(rosplane_controller rosplane_msgs rosflight_msgs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane_controller
  DESTINATION lib/${PROJECT_NAME})


# Follower
add_executable(rosplane_path_follower
  src/path_follower_example.cpp
  src/path_follower_base.cpp
  src/param_manager.cpp)
ament_target_dependencies(rosplane_path_follower rosplane_msgs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane_path_follower
  DESTINATION lib/${PROJECT_NAME})

# Manager
add_executable(rosplane_path_manager
  src/path_manager_base.cpp
  src/path_manager_example.cpp
  src/param_manager.cpp)
ament_target_dependencies(rosplane_path_manager rosplane_msgs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane_path_manager
  DESTINATION lib/${PROJECT_NAME})

# Planner
add_executable(rosplane_path_planner
  src/path_planner.cpp
  src/param_manager.cpp)
target_link_libraries(rosplane_path_planner
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(rosplane_path_planner rosplane_msgs rosflight_msgs std_srvs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane_path_planner
  DESTINATION lib/${PROJECT_NAME})

# Estimator
add_executable(rosplane_estimator_node
  src/estimator_base.cpp
  src/estimator_example.cpp
  src/param_manager.cpp)
ament_target_dependencies(rosplane_estimator_node rosplane_msgs rosflight_msgs rclcpp Eigen3)
install(TARGETS
  rosplane_estimator_node
  DESTINATION lib/${PROJECT_NAME})

# Input Mapper
add_executable(input_mapper
  src/input_mapper.cpp
  src/param_manager.cpp)
ament_target_dependencies(input_mapper rosplane_msgs rosflight_msgs rclcpp std_srvs)
install(TARGETS
  input_mapper
  DESTINATION lib/${PROJECT_NAME})

#### END OF EXECUTABLES ###


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
