cmake_minimum_required(VERSION 3.20...3.28)
project(rosplane_tuning)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

###################
### Executables ###
###################

# Signal Generator
add_executable(signal_generator src/signal_generator.cpp)
target_compile_features(signal_generator PRIVATE cxx_std_17)
target_compile_options(signal_generator PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(signal_generator PRIVATE include)
target_link_libraries(signal_generator PRIVATE
  rclcpp::rclcpp
  std_srvs::std_srvs
  rosplane_msgs::rosplane_msgs
)

###############
### Install ###
###############

# C++ nodes/executables
install(
  TARGETS
    signal_generator
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    resources
  DESTINATION share/${PROJECT_NAME}
)

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
