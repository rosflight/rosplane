cmake_minimum_required(VERSION 3.20...3.28)
project(rosplane)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)
find_package(rosflight_compat REQUIRED)
find_package(yaml-cpp REQUIRED)

# FIXME: Legacy yaml-cpp support for Ubuntu 22.04
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
  set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${YAML_CPP_LIBRARIES}"
  )
endif()

#################
### Libraries ###
#################

# Param Manager (exported)
add_library(rosplane_param_manager src/param_manager/param_manager.cpp)
add_library(rosplane::param_manager ALIAS rosplane_param_manager)
set_target_properties(rosplane_param_manager PROPERTIES
  EXPORT_NAME param_manager
)
target_compile_features(rosplane_param_manager PUBLIC cxx_std_17)
target_compile_options(rosplane_param_manager PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rosplane_param_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(rosplane_param_manager PRIVATE rclcpp::rclcpp)

###################
### Executables ###
###################

# Controller
add_executable(controller
  src/controller/controller_ros.cpp
  src/controller/controller_state_machine.cpp
  src/controller/controller_successive_loop.cpp
  src/controller/controller_total_energy.cpp
)
target_compile_features(controller PRIVATE cxx_std_17)
target_compile_options(controller PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(controller PRIVATE include)
target_link_libraries(controller PRIVATE
  rosplane::param_manager
  rclcpp::rclcpp
  rosplane_msgs::rosplane_msgs
  rosflight_msgs::rosflight_msgs
  Eigen3::Eigen
)

# Path Follower
add_executable(path_follower
  src/path_follower/path_follower_lines_orbits.cpp
  src/path_follower/path_follower_ros.cpp
)
target_compile_features(path_follower PRIVATE cxx_std_17)
target_compile_options(path_follower PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(path_follower PRIVATE include)
target_link_libraries(path_follower PRIVATE
  rosplane::param_manager
  rclcpp::rclcpp
  rosplane_msgs::rosplane_msgs
  Eigen3::Eigen
)

# Path Manager
add_executable(path_manager
  src/path_manager/path_manager_ros.cpp
  src/path_manager/path_manager_dubins_fillets.cpp
)
target_compile_features(path_manager PRIVATE cxx_std_17)
target_compile_options(path_manager PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(path_manager PRIVATE include)
target_link_libraries(path_manager PRIVATE
  rosplane::param_manager
  rclcpp::rclcpp
  rosplane_msgs::rosplane_msgs
  Eigen3::Eigen
)

# Path Planner
add_executable(path_planner src/path_planner/path_planner_waypoints.cpp)
target_compile_features(path_planner PRIVATE cxx_std_17)
target_compile_options(path_planner PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(path_planner PRIVATE include)
target_link_libraries(path_planner PRIVATE
  rosplane::param_manager
  rclcpp::rclcpp
  std_srvs::std_srvs
  rosflight_msgs::rosflight_msgs
  rosplane_msgs::rosplane_msgs
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
)

# Estimator
set_source_files_properties(geomag.c PROPERTIES LANGUAGE C)
add_executable(estimator
  src/ekf/estimator_node.cpp
  src/ekf/estimator_ros.cpp
  src/ekf/estimator_ekf.cpp
  src/ekf/estimator_continuous_discrete.cpp
  src/utils/geomag.c
)
target_compile_features(estimator PRIVATE cxx_std_17)
target_compile_options(estimator PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(estimator PRIVATE include)
target_link_libraries(estimator PRIVATE
  rosplane::param_manager
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  rosflight_compat::compat_headers # FIXME: remove when Humble is dropped
  sensor_msgs::sensor_msgs
  rosflight_msgs::rosflight_msgs
  rosplane_msgs::rosplane_msgs
  Eigen3::Eigen
)

###############
### Install ###
###############

# Headers for exported/public libraries
install(
  DIRECTORY
    include/param_manager
  DESTINATION include/${PROJECT_NAME}
)

# Exported/public libraries
install(
  TARGETS
    rosplane_param_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# C++ nodes/executables
install(
  TARGETS
    controller
    path_follower
    path_manager
    path_planner
    estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(
  DIRECTORY
    launch
    params
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

# Export custom libraries
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export public CMake package dependencies (not targets)
ament_export_dependencies(
  rclcpp
)

ament_package()
