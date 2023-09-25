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

    node->declare_parameter("joint0", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("joint1", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("joint2", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("joint3", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("joint4", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("joint5", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("joint6", rclcpp::PARAMETER_DOUBLE);
    std::vector<std::string> joints = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    // std::vector<rclcpp::Parameter> params = node->get_parameters(joints);
    // for (auto &param : params){
    //     RCLCPP_INFO(logger, "%s: %s",
    //                 param.get_name().c_str(), param.value_to_string().c_str());
    // }


    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt Move Group Interface for panda arm
    moveit::planning_interface::MoveGroupInterface move_group_xarm(node, "xarm7");
    moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "xarm_gripper");

    // Get all joint positions
    std::vector<double> joint_group_positions = move_group_xarm.getCurrentJointValues();

    for (int i=0; i<7; ++i){
        RCLCPP_INFO(logger, "Joint %d: %f", i, joint_group_positions[i]);
        node->set_parameters(std::vector<rclcpp::Parameter>{rclcpp::Parameter(joints[i], joint_group_positions[i])});
    }

    geometry_msgs::msg::PoseStamped xarm_pose = move_group_xarm.getCurrentPose();
    RCLCPP_INFO(logger, "position x: %f", xarm_pose.pose.position.x);
    RCLCPP_INFO(logger, "position y: %f", xarm_pose.pose.position.y);
    RCLCPP_INFO(logger, "position z: %f", xarm_pose.pose.position.z);
    RCLCPP_INFO(logger, "orientation w: %f", xarm_pose.pose.orientation.w);
    RCLCPP_INFO(logger, "orientation x: %f", xarm_pose.pose.orientation.x);
    RCLCPP_INFO(logger, "orientation y: %f", xarm_pose.pose.orientation.y);
    RCLCPP_INFO(logger, "orientation z: %f", xarm_pose.pose.orientation.z);

    std::vector<double> gripper_positions = move_group_gripper.getCurrentJointValues();
    RCLCPP_INFO(logger, "Gripper Joint: %f", gripper_positions[1]);
    
    // rclcpp::spin(node);

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}