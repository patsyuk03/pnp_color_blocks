cmake_minimum_required(VERSION 3.8)
project(pnp_color_blocks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(pnp src/pnp.cpp)
ament_target_dependencies(
  pnp
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
)
add_executable(pnp_try src/pnp_try.cpp)
ament_target_dependencies(
  pnp_try
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
)
add_executable(get_xarm_pose src/get_xarm_pose.cpp)
ament_target_dependencies(
  get_xarm_pose
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
)

install(TARGETS 
  pnp get_xarm_pose pnp_try
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  yaml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()