#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Create a ROS logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pnp");

int main(int argc, char * argv[]){
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pnp_node");
    RCLCPP_INFO(LOGGER, "Initialize node");

    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    RCLCPP_INFO(LOGGER, "Created executor");

    // Create the MoveIt Move Group Interface for xArm7
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, "xarm7");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// Adding table
    auto const add_object1 = [frame_id = move_group.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject add_object1;
        add_object1.header.frame_id = frame_id;
        add_object1.id = "table";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.5;
        primitive.dimensions[primitive.BOX_Y] = 1.5;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = 0.5;
        table_pose.position.y = 0.0;
        table_pose.position.z = -0.1;

        add_object1.primitives.push_back(primitive);
        add_object1.primitive_poses.push_back(table_pose);
        add_object1.operation = add_object1.ADD;

        return add_object1;}();
    planning_scene_interface.applyCollisionObject(add_object1);

// Go to initial position
    move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));

    // Create a plan to that target pose and check if that plan is successful
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success){move_group.execute(my_plan);}
    RCLCPP_INFO(LOGGER, "Executing plan 0 (home position) %s", success ? "" : "FAILED");
}