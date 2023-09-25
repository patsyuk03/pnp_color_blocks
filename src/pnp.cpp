#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<double> initial_pose = {-0.097065, -0.842345, -0.005829, 0.710569, -0.014803, 1.588374, 0.000941};

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("pnp_node");

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("pnp_node");

    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt Move Group Interface for xarm and gripper
    moveit::planning_interface::MoveGroupInterface move_group_xarm(node, "xarm7");
    moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "xarm_gripper");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction;

    geometry_msgs::msg::PoseStamped xarm_pose;

    geometry_msgs::msg::PoseStamped block_pose;
    block_pose.pose.position.x = 0.422677;
    block_pose.pose.position.y = -0.058144;
    block_pose.pose.position.z = 0.004785;
    block_pose.pose.orientation.x = 0.997906;
    block_pose.pose.orientation.y = -0.058829;
    block_pose.pose.orientation.z = 0.025479;
    block_pose.pose.orientation.w = 0.008559;

    geometry_msgs::msg::PoseStamped box_pose;
    box_pose.pose.position.x = 0.176080;
    box_pose.pose.position.y = -0.330890;
    box_pose.pose.position.z = 0.169716;
    box_pose.pose.orientation.x = 0.902038;
    box_pose.pose.orientation.y = -0.430438;
    box_pose.pose.orientation.z = 0.011106;
    box_pose.pose.orientation.w = 0.030469;

    // Move to initial position
    move_group_xarm.setJointValueTarget(initial_pose);
    bool success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_xarm.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Gripper open
    move_group_gripper.setJointValueTarget("drive_joint", 0.0);
    success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_gripper.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move to block
    xarm_pose = move_group_xarm.getCurrentPose();
    xarm_pose.pose.position.x = block_pose.pose.position.x;
    xarm_pose.pose.position.y = block_pose.pose.position.y;
    xarm_pose.pose.position.z = block_pose.pose.position.z+0.2;
    xarm_pose.pose.orientation.x = block_pose.pose.orientation.x;
    xarm_pose.pose.orientation.y = block_pose.pose.orientation.y;
    xarm_pose.pose.orientation.z = block_pose.pose.orientation.z;
    xarm_pose.pose.orientation.w = block_pose.pose.orientation.w;

    waypoints = {};
    waypoints.push_back(xarm_pose.pose);
    fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Move down
    waypoints = {};
    xarm_pose.pose.position.z = xarm_pose.pose.position.z-0.2;
    waypoints.push_back(xarm_pose.pose);
    fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Close gripper
    move_group_gripper.setJointValueTarget("drive_joint", 0.58);
    success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_gripper.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move up
    waypoints = {};
    xarm_pose.pose.position.z = xarm_pose.pose.position.z+0.2;
    waypoints.push_back(xarm_pose.pose);
    fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Move to initial position
    move_group_xarm.setJointValueTarget(initial_pose);
    success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_xarm.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move to box
    xarm_pose = move_group_xarm.getCurrentPose();
    xarm_pose.pose.position.x = box_pose.pose.position.x;
    xarm_pose.pose.position.y = box_pose.pose.position.y;
    xarm_pose.pose.position.z = box_pose.pose.position.z+0.2;
    xarm_pose.pose.orientation.x = box_pose.pose.orientation.x;
    xarm_pose.pose.orientation.y = box_pose.pose.orientation.y;
    xarm_pose.pose.orientation.z = box_pose.pose.orientation.z;
    xarm_pose.pose.orientation.w = box_pose.pose.orientation.w;

    waypoints = {};
    waypoints.push_back(xarm_pose.pose);
    fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Move down
    waypoints = {};
    xarm_pose.pose.position.z = xarm_pose.pose.position.z-0.2;
    waypoints.push_back(xarm_pose.pose);
    fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Gripper open
    move_group_gripper.setJointValueTarget("drive_joint", 0.0);
    success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_gripper.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move up
    waypoints = {};
    xarm_pose.pose.position.z = xarm_pose.pose.position.z+0.2;
    waypoints.push_back(xarm_pose.pose);
    fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Move to initial position
    move_group_xarm.setJointValueTarget(initial_pose);
    success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_xarm.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}