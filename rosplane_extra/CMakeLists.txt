cmake_minimum_required(VERSION 3.8)
project(rosplane_extra)

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
find_package(rosplane)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosplane_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)

ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

include_directories(include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

### START OF REAL EXECUTABLES ###

# Input Mapper
add_executable(input_mapper
  src/input_mapper.cpp
  src/param_manager.cpp)
ament_target_dependencies(input_mapper rosplane_msgs rosflight_msgs rclcpp std_srvs)
install(TARGETS
  input_mapper
  DESTINATION lib/${PROJECT_NAME})

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
