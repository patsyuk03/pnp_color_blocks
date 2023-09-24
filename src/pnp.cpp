#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<double> initial_pose = {-0.611977, -0.824699, 0.035698, 0.799772, 0.026266, 1.624083, 0.075338};

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

    // Move to initial position
    move_group_xarm.setJointValueTarget(initial_pose);
    bool success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_xarm.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Gripper open
    move_group_gripper.setJointValueTarget({0.0});
    success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_gripper.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move to block
    geometry_msgs::msg::PoseStamped block_pose;
    block_pose.pose.position.x = 0.29;
    block_pose.pose.position.y = -0.30;
    block_pose.pose.position.z = 0.06;

    geometry_msgs::msg::PoseStamped xarm_pose = move_group_xarm.getCurrentPose();
    xarm_pose.pose.position.x = block_pose.pose.position.x;
    xarm_pose.pose.position.y = block_pose.pose.position.y;
    xarm_pose.pose.position.z = block_pose.pose.position.z+0.2;

    move_group_xarm.setPoseTarget(xarm_pose);
    success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) {
        move_group_xarm.execute(my_plan);
    } else {
        RCLCPP_ERROR(logger, "Planing failed!");
    }

    // Move down
    std::vector<geometry_msgs::msg::Pose> waypoints;
    xarm_pose.pose.position.z = xarm_pose.pose.position.z-0.2;
    waypoints.push_back(xarm_pose.pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
    if(fraction == 1){
        move_group_xarm.execute(trajectory);
    }

    // Close gripper
    move_group_gripper.setJointValueTarget("drive_joint", 0.8);
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