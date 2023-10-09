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
find_package(ros2_aruco_interfaces REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gripper_srv REQUIRED)

add_executable(pnp_xarm src/pnp_xarm.cpp)
ament_target_dependencies(
  pnp_xarm
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

add_executable(pnp_ur src/pnp_ur.cpp)
ament_target_dependencies(
  pnp_ur
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "gripper_srv"
)

add_executable(pnp_ur_no_gripper src/pnp_ur_no_gripper.cpp)
ament_target_dependencies(
  pnp_ur_no_gripper
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

add_executable(test_robotiq_gripper src/test_robotiq_gripper.cpp)
ament_target_dependencies(
  test_robotiq_gripper
  "rclcpp"
  "gripper_srv"
)

add_executable(pnp_try src/pnp_try.cpp)
ament_target_dependencies(
  pnp_try
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
)
add_executable(transform_pose_marker src/transform_pose_marker.cpp)
ament_target_dependencies(
  transform_pose_marker
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)
add_executable(get_robot_pose src/get_robot_pose.cpp)
ament_target_dependencies(
  get_robot_pose
  "rclcpp"
  "moveit_msgs"
  "moveit_ros_planning_interface"
)

install(TARGETS 
  pnp_xarm pnp_ur get_robot_pose pnp_try transform_pose_marker test_robotiq_gripper pnp_ur_no_gripper
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