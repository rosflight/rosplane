cmake_minimum_required(VERSION 3.20...3.28)
project(rosplane_gcs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

###################
### Executables ###
###################

# Publisher node executable
add_executable(rviz_waypoint_publisher src/rviz_waypoint_publisher.cpp)
target_compile_features(rviz_waypoint_publisher PRIVATE cxx_std_17)
target_compile_options(rviz_waypoint_publisher PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(rviz_waypoint_publisher PRIVATE
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  visualization_msgs::visualization_msgs
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  rosplane_msgs::rosplane_msgs
)

###############
### Install ###
###############

# C++ nodes/executables
install(
  TARGETS
    rviz_waypoint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    config
    resource
  DESTINATION share/${PROJECT_NAME}
)

########################
### ROS Finalization ###
########################

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
