cmake_minimum_required(VERSION 3.8)
project(rosplane2)

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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosplane2_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosflight_msgs REQUIRED)

ament_export_dependencies(rclcpp rclpy)
ament_export_include_directories(include)

#install(DIRECTORY originalFiles DESTINATION share/${PROJECT_NAME}/)
include_directories(  #use this if you need .h files for include statements.  The include will need to have the directories where each .h is respectively.
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

### START OF REAL EXECUTABLES ###

# Truth Node
add_executable(rosplane2_truth_publisher
            src/aircraft_truth.cpp)
ament_target_dependencies(rosplane2_truth_publisher rosplane2_msgs rclcpp rclpy nav_msgs)
install(TARGETS
  rosplane2_truth_publisher
  DESTINATION lib/${PROJECT_NAME})

# Forces Node
add_executable(rosplane2_force_publisher
            src/forces_moments.cpp)
ament_target_dependencies(rosplane2_force_publisher rosplane2_msgs geometry_msgs rclcpp rclpy nav_msgs)
install(TARGETS
  rosplane2_force_publisher
  DESTINATION lib/${PROJECT_NAME})

# Autopilot
add_executable(rosplane2_controller
            src/controller_base.cpp
            src/controller_example.cpp)
ament_target_dependencies(rosplane2_controller rosplane2_msgs rosflight_msgs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane2_controller
  DESTINATION lib/${PROJECT_NAME})

# Estimator
#add_executable(rosplane2_estimator
#            originalFiles/estimator_base.cpp
#            originalFiles/estimator_example.cpp)
#ament_target_dependencies(rosplane2_estimator rosplane2_msgs rclcpp rclpy Eigen3)
#install(TARGETS
#  rosplane2_estimator
#  DESTINATION lib/${PROJECT_NAME})

# Follower
add_executable(rosplane2_path_follower
            src/path_follower_example.cpp
            src/path_follower_base.cpp)
ament_target_dependencies(rosplane2_path_follower rosplane2_msgs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane2_path_follower
  DESTINATION lib/${PROJECT_NAME})

# Manager
add_executable(rosplane2_path_manager
            src/path_manager_base.cpp
            src/path_manager_example.cpp)
ament_target_dependencies(rosplane2_path_manager rosplane2_msgs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane2_path_manager
  DESTINATION lib/${PROJECT_NAME})

# Planner
add_executable(rosplane2_path_planner
            src/path_planner.cpp)
ament_target_dependencies(rosplane2_path_planner rosplane2_msgs rclcpp rclpy Eigen3)
install(TARGETS
  rosplane2_path_planner
  DESTINATION lib/${PROJECT_NAME})

add_executable(rosplane2_transform_node
            src/transform_node.cpp)
ament_target_dependencies(rosplane2_transform_node rosplane2_msgs rclcpp rclpy Eigen3 tf2 tf2_ros)
install(TARGETS
        rosplane2_transform_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(rosplane2_estimator_node
              src/estimator_base.cpp
              src/estimator_example.cpp)
ament_target_dependencies(rosplane2_estimator_node rosplane2_msgs rosflight_msgs rclcpp Eigen3)
install(TARGETS
        rosplane2_estimator_node
        DESTINATION lib/${PROJECT_NAME})
        
add_executable(rosplane2_sensors_node
            src/sensors.cpp)

ament_target_dependencies(rosplane2_sensors_node rosplane2_msgs rosflight_msgs rclcpp rclpy nav_msgs Eigen3)
install(TARGETS
        rosplane2_sensors_node
        DESTINATION lib/${PROJECT_NAME})


#### END OF REAL EXECUTABLES ###


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
