#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose_array.hpp"
// #include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

using std::placeholders::_1;
std::vector<double> initial_pose = {-0.611977, -0.824699, 0.035698, 0.799772, 0.026266, 1.624083, 0.075338};


class PnPBlocks : public rclcpp::Node {
    public:
    PnPBlocks(): Node("pnp")
    {
        auto move_group_node = rclcpp::Node::make_shared("move_group_node");
        // sub = this->create_subscription<geometry_msgs::msg::PoseArray>("aruco_poses", 1000, std::bind(&PnPBlocks::callback, this, _1));

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        auto spinner = std::thread([&executor]() { executor.spin(); });

        // Create the MoveIt Move Group Interface for xarm and gripper
        using moveit::planning_interface::MoveGroupInterface;
        move_group_xarm = MoveGroupInterface(move_group_node, "xarm7");
        move_group_gripper = MoveGroupInterface(move_group_node, "xarm_gripper");
        // moveit::planning_interface::MoveGroupInterface move_group_xarm(move_group_node, "xarm7");
        // moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, "xarm_gripper");
        // move_group_xarm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "xarm7");
        // move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "xarm_gripper");

        block_pose.pose.position.x = 0.29;
        block_pose.pose.position.y = -0.30;
        block_pose.pose.position.z = 0.06;

        initial_position();
        gripper_state(0.0);
        pose_goal(block_pose);
        cartesian_path(-0.2);
        gripper_state(0.8);
        cartesian_path(0.2);
        initial_position();
    }
    private:
    void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        geometry_msgs::msg::PoseArray marker_pose;            
        marker_pose = *msg;
        if (!marker_pose.poses.empty()){
            block_pose.pose.position.x = marker_pose.poses[0].position.x;
            block_pose.pose.position.y = marker_pose.poses[0].position.y;
            block_pose.pose.position.z = marker_pose.poses[0].position.z+0.2;

            block_pose.pose.orientation.x = marker_pose.poses[0].orientation.x;
            block_pose.pose.orientation.y = marker_pose.poses[0].orientation.y;
            block_pose.pose.orientation.z = marker_pose.poses[0].orientation.z;	
            block_pose.pose.orientation.w = marker_pose.poses[0].orientation.w;	
        };
    }
    void initial_position() {
        move_group_xarm.setJointValueTarget(initial_pose);
        success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_xarm.execute(my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        }
    }
    void gripper_state(double state) {
        move_group_gripper.setJointValueTarget("drive_joint", state);
        success = (move_group_gripper.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_gripper.execute(my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        }
    }
    void pose_goal(geometry_msgs::msg::PoseStamped new_pose) {
        move_group_xarm.setPoseTarget(new_pose);
        success = (move_group_xarm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(success) {
            move_group_xarm.execute(my_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planing failed!");
        }
    }
    void cartesian_path(double state) {
        waypoints = {};
        xarm_pose = move_group_xarm.getCurrentPose();
        xarm_pose.pose.position.z = xarm_pose.pose.position.z+state;
        waypoints.push_back(xarm_pose.pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_xarm.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
        RCLCPP_INFO(this->get_logger(), "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);
        if(fraction == 1){
            move_group_xarm.execute(trajectory);
        }
    }
    // rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub;
    // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_xarm;
    // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper;
    // moveit::planning_interface::MoveGroupInterfacePtr move_group_xarm_;
    // moveit::planning_interface::MoveGroupInterfacePtr move_group_gripper_;
    moveit::planning_interface::MoveGroupInterface move_group_xarm;
    moveit::planning_interface::MoveGroupInterface move_group_gripper;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::msg::PoseStamped block_pose;
    geometry_msgs::msg::PoseStamped xarm_pose;
    bool success;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    // rclcpp::Node::SharedPtr node_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PnPBlocks>();
  rclcpp::shutdown();
  return 0;
}