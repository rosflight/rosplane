cmake_minimum_required(VERSION 3.20...3.28)
project(rosplane_extra)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosplane REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)

###################
### Executables ###
###################

# Input Mapper
add_executable(input_mapper src/input_mapper.cpp)
target_compile_features(input_mapper PRIVATE cxx_std_17)
target_compile_options(input_mapper PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(input_mapper PRIVATE include)
target_link_libraries(input_mapper PRIVATE
  rclcpp::rclcpp
  rosplane::param_manager
  std_srvs::std_srvs
  rosplane_msgs::rosplane_msgs
  rosflight_msgs::rosflight_msgs
)

###############
### Install ###
###############

# C++ nodes/executables
install(
  TARGETS
    input_mapper
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

ament_package()
