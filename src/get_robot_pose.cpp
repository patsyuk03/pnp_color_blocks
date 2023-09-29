#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("get_pose_node");

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("get_pose_node");

    node->declare_parameter("robot", rclcpp::PARAMETER_STRING);

    std::vector<std::string> xarm_joints = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    std::vector<std::string> ur_joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto robot = node->get_parameter("robot");

    // Create the MoveIt Move Group Interface for panda arm
    moveit::planning_interface::MoveGroupInterface move_group(node, robot.value_to_string().c_str());
    if (robot.value_to_string() == "xarm7"){
        moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "xarm_gripper");
        std::vector<double> gripper_positions = move_group_gripper.getCurrentJointValues();
        RCLCPP_INFO(logger, "Gripper Joint: %f", gripper_positions[1]);
    }

    std::vector<std::string> joints;
    if (robot.value_to_string() == "xarm7"){
        joints = xarm_joints;
    } else {
        joints = ur_joints;
    }
    
    // Get all joint positions
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();

    for (int i=0; i<joints.size(); ++i){
        RCLCPP_INFO(logger, "%s: %f", joints[i].c_str(), joint_group_positions[i]);
    }

    geometry_msgs::msg::PoseStamped robot_pose = move_group.getCurrentPose();
    RCLCPP_INFO(logger, "position x: %f", robot_pose.pose.position.x);
    RCLCPP_INFO(logger, "position y: %f", robot_pose.pose.position.y);
    RCLCPP_INFO(logger, "position z: %f", robot_pose.pose.position.z);
    RCLCPP_INFO(logger, "orientation w: %f", robot_pose.pose.orientation.w);
    RCLCPP_INFO(logger, "orientation x: %f", robot_pose.pose.orientation.x);
    RCLCPP_INFO(logger, "orientation y: %f", robot_pose.pose.orientation.y);
    RCLCPP_INFO(logger, "orientation z: %f", robot_pose.pose.orientation.z);

    
    // rclcpp::spin(node);

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}