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

include_directories(
  include
  include/param_manager
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

### EXTRA FILES TO INSTALL ###

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)

### LIBRARIES ###

# Param Manager
add_library(param_manager
  include/param_manager/param_manager.hpp
  src/param_manager/param_manager.cpp
)
ament_target_dependencies(param_manager rclcpp)
ament_export_targets(param_manager HAS_LIBRARY_TARGET)
install(DIRECTORY include/param_manager DESTINATION include)
install(TARGETS param_manager
  EXPORT param_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

### START OF EXECUTABLES ###

# Controller
add_executable(controller
  src/controller/controller_ros.cpp
  src/controller/controller_state_machine.cpp
  src/controller/controller_successive_loop.cpp
  src/controller/controller_total_energy.cpp)
ament_target_dependencies(controller rosplane_msgs rosflight_msgs rclcpp rclpy Eigen3)
target_link_libraries(controller param_manager)
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME})

# Follower
add_executable(path_follower
  src/path_follower/path_follower_lines_orbits.cpp
  src/path_follower/path_follower_ros.cpp)
ament_target_dependencies(path_follower rosplane_msgs rclcpp rclpy Eigen3)
target_link_libraries(path_follower param_manager)
install(TARGETS
  path_follower
  DESTINATION lib/${PROJECT_NAME})

# Manager
add_executable(path_manager
  src/path_manager/path_manager_ros.cpp
  src/path_manager/path_manager_dubins_fillets.cpp)
ament_target_dependencies(path_manager rosplane_msgs rclcpp rclpy Eigen3)
target_link_libraries(path_manager param_manager)
install(TARGETS
  path_manager
  DESTINATION lib/${PROJECT_NAME})

# Planner
add_executable(path_planner
  src/path_planner/path_planner_waypoints.cpp)
target_link_libraries(path_planner
  param_manager
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(path_planner rosplane_msgs rosflight_msgs std_srvs rclcpp rclpy Eigen3)
install(TARGETS
  path_planner
  DESTINATION lib/${PROJECT_NAME})

# Estimator
set_source_files_properties(geomag.c PROPERTIES LANGUAGE C)
add_executable(estimator
              src/ekf/estimator_node.cpp
              src/ekf/estimator_ros.cpp
              src/ekf/estimator_ekf.cpp
              src/ekf/estimator_continuous_discrete.cpp
              src/utils/geomag.c)
target_link_libraries(estimator
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(estimator rosplane_msgs rosflight_msgs sensor_msgs rclcpp Eigen3 ament_index_cpp)
target_link_libraries(estimator param_manager)
target_include_directories(estimator PUBLIC include)
install(TARGETS
  estimator
  DESTINATION lib/${PROJECT_NAME})

add_library(estimator_lib
  src/ekf/estimator_node.cpp
  src/ekf/estimator_ros.cpp
  src/ekf/estimator_ekf.cpp
  src/ekf/estimator_continuous_discrete.cpp
  src/utils/geomag.c
  include/ekf/estimator_ros.hpp
  include/ekf/estimator_ekf.hpp
  include/ekf/estimator_continuous_discrete.hpp)
target_link_libraries(estimator_lib param_manager)
target_link_libraries(estimator estimator_lib)
ament_target_dependencies(estimator_lib rclcpp rosflight_msgs sensor_msgs Eigen3 rosplane_msgs)

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
