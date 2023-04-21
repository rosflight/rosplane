cmake_minimum_required(VERSION 3.8)
project(first_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosplane2_msgs REQUIRED)



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

# add executables
add_executable(talker src/test_gazebo_pub.cpp)
ament_target_dependencies(talker rclcpp nav_msgs rosplane2_msgs)

add_executable(listener src/test_aircraft_truth.cpp)
ament_target_dependencies(listener rclcpp nav_msgs rosplane2_msgs)

# install section
install(TARGETS
talker
listener
DESTINATION lib/${PROJECT_NAME})


ament_package()
