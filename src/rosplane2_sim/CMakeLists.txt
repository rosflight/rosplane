cmake_minimum_required(VERSION 3.8)
project(rosplane2_sim)

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
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(rosflight_msgs REQUIRED)
# find_package(rosplane_msgs REQUIRED)
# find_package(gazebo_plugins REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY Aircraft  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME}/)

# include_directories(
#   include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${GAZEBO_INCLUDE_DIRS}
# )

# link_directories(${GAZEBO_LIBRARY_DIRS}) # gz-cmake???

ament_export_dependencies(
  rclcpp
  geometry_msgs
  ros_gz
  #gazebo_plugins
  #rosflight_msgs 
  #rosplane_msgs
  Eigen3
  #gazebo???
)

# add_library(aircraft_truth_plugin src/aircraft_truth.cpp)
# target_link_libraries(aircraft_truth_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_dependencies(aircraft_truth_plugin ${catkin_EXPORTED_TARGETS})

# add_library(aircraft_forces_and_moments_plugin src/aircraft_forces_and_moments.cpp)
# target_link_libraries(aircraft_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_dependencies(aircraft_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

# ament_export_include_directories(include)
# ament_export_libraries()

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
