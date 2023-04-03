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
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rosplane2 REQUIRED)

ament_export_dependencies(rclcpp rclpy)
ament_export_include_directories(include)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
  install(DIRECTORY originalFiles DESTINATION share/${PROJECT_NAME}/)


include_directories(  #use this if you need .h files for include statements.  The include will need to have the directories where each .h is respectively.
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

#add_executable(rosplane_controller
#            originalFiles/controller_base.cpp
#            originalFiles/controller_example.cpp)

#ament_target_dependencies(rosplane_controller rclcpp rclpy Eigen3)
#target_link_libraries(rosplane_controller PLANE_HEADERS)

#install(TARGETS
#  rosplane_controller
#  DESTINATION lib/${PROJECT_NAME})

add_executable(talker originalFiles/testNode.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
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
