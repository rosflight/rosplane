cmake_minimum_required(VERSION 3.20...3.28)
project(rosplane_sim)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosflight_compat REQUIRED) # FIXME: remove when Humble is dropped
find_package(rosflight_msgs REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

###################
### Executables ###
###################

add_executable(sim_state_transcriber src/sim_state_transcription.cpp)
target_compile_features(sim_state_transcriber PRIVATE cxx_std_17)
target_compile_options(sim_state_transcriber PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(sim_state_transcriber PRIVATE
  rclcpp::rclcpp
  rosflight_compat::compat_headers # FIXME: remove when Humble is dropped
  rosflight_msgs::rosflight_msgs
  rosplane_msgs::rosplane_msgs
  Eigen3::Eigen
)

###############
### Install ###
###############

# C++ nodes/executables
install(
  TARGETS
    sim_state_transcriber
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
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
