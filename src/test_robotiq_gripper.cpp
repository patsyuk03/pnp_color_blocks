#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "gripper_srv/srv/gripper_service.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = rclcpp::Node::make_shared("gripper_test_node", options);

    RCLCPP_INFO(node->get_logger(), "Using service");

    auto gripper_service_client = node->create_client<gripper_srv::srv::GripperService>("gripper_service");

    while (!gripper_service_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();

    // Open gripper
    request->position = 0;
    request->speed = 255;
    request->force = 255;
    auto response = gripper_service_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, response);

    // Close gripper
    request->position = 110;
    request->speed = 255;
    request->force = 255;
    response = gripper_service_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, response);
/*
    // Open gripper small speed
    request->position = 0;
    request->speed = 55;
    request->force = 255;
    response = gripper_service_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, response);

    // Close gripper small speed
    request->position = 255;
    request->speed = 55;
    request->force = 255;
    response = gripper_service_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, response);

    // Open gripper small speed and force
    request->position = 100;
    request->speed = 5;
    request->force = 5;
    response = gripper_service_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, response);

    // Close gripper small speed and force
    request->position = 150;
    request->speed = 5;
    request->force = 5;
    response = gripper_service_client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, response);
*/
    rclcpp::shutdown();
    return 0;
}