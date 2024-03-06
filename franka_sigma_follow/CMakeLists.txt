cmake_minimum_required(VERSION 3.8)
project(franka_sigma_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(franka_target_publisher src/franka_target_publisher.cpp)
add_executable(franka_follow_sigma src/franka_follow_sigma.cpp)

target_include_directories(franka_target_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(franka_target_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  franka_target_publisher
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "moveit"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)
ament_target_dependencies(
  franka_follow_sigma
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "moveit"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)

install(TARGETS franka_target_publisher
  franka_follow_sigma
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
