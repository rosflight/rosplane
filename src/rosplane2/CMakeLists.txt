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

#START OF REAL EXECUTABLES
#add_executable(rosplane_controller
#            originalFiles/controller_base.cpp
#            originalFiles/controller_example.cpp)

#ament_target_dependencies(rosplane_controller rosplane2_msgs rclcpp rclpy Eigen3)

#install(TARGETS
#  rosplane_controller
#  DESTINATION lib/${PROJECT_NAME})


#add_executable(rosplane_estimator
#            originalFiles/estimator_base.cpp
#            originalFiles/estimator_example.cpp)

#ament_target_dependencies(rosplane_estimator rosplane2_msgs rclcpp rclpy Eigen3)

#install(TARGETS
#  rosplane_estimator
#  DESTINATION lib/${PROJECT_NAME})



#add_executable(rosplane_path_follower
#            originalFiles/path_follower_example.cpp
#            originalFiles/path_follower_base.cpp)

#ament_target_dependencies(rosplane_path_follower rosplane2_msgs rclcpp rclpy Eigen3)

#install(TARGETS
#  rosplane_path_follower
#  DESTINATION lib/${PROJECT_NAME})



add_executable(rosplane_path_manager
            originalFiles/path_manager_base.cpp
            originalFiles/path_manager_example.cpp)

ament_target_dependencies(rosplane_path_manager rosplane2_msgs rclcpp rclpy Eigen3)

install(TARGETS
  rosplane_path_manager
  DESTINATION lib/${PROJECT_NAME})



add_executable(rosplane_path_planner
            originalFiles/path_planner.cpp)

ament_target_dependencies(rosplane_path_planner rosplane2_msgs rclcpp rclpy Eigen3)

install(TARGETS
  rosplane_path_planner
  DESTINATION lib/${PROJECT_NAME})

# END OF REAL EXECUTABLES

#custom executable, just as sanity check to make sure it works.
add_executable(talker originalFiles/testNode.cpp)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)

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
