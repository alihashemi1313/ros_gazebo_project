cmake_minimum_required(VERSION 3.8)
project(motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(position_target src/position_target.cpp)
target_include_directories(position_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(position_target PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  position_target
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
)

add_executable(position_subscriber src/position_subscriber.cpp)
ament_target_dependencies(position_subscriber rclcpp geometry_msgs moveit_ros_planning_interface)

add_executable(waypoints_target src/waypoints_target.cpp)
ament_target_dependencies(waypoints_target rclcpp geometry_msgs moveit_ros_planning_interface)

install(TARGETS
  position_target
  position_subscriber
  waypoints_target
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
