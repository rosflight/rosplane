cmake_minimum_required(VERSION 3.8)
project(rosplane_tuning)

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
find_package(ament_cmake_python REQUIRED)
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

ament_export_dependencies(rclcpp rclpy)
ament_export_include_directories(include)

#install(DIRECTORY originalFiles DESTINATION share/${PROJECT_NAME}/)
include_directories(  #use this if you need .h files for include statements.  The include will need to have the directories where each .h is respectively.
  include
  ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

### START OF EXECUTABLES ###

# Signal Generator
add_executable(signal_generator
               src/signal_generator.cpp)
ament_target_dependencies(signal_generator rosplane_msgs std_srvs rclcpp)
target_compile_options(signal_generator PRIVATE -Wno-unused-parameter)
install(TARGETS
        signal_generator
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
