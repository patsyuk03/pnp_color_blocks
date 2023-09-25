#include <memory>
#include <rclcpp/rclcpp.hpp>
// #include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;


class TransformPoseMarker : public rclcpp::Node
{
    public:
    TransformPoseMarker()
    : Node("transform_pose_marker")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        sub = this->create_subscription<geometry_msgs::msg::PoseArray>("aruco_poses", 100, std::bind(&TransformPoseMarker::callback, this, _1));
        pub = this->create_publisher<geometry_msgs::msg::PoseArray>("tf_aruco_poses", 1);   
        RCLCPP_INFO(this->get_logger(), "Subscribing to aruco_poses");     


    }
    private:
    void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {

        geometry_msgs::msg::PoseStamped marker_pose;  
        geometry_msgs::msg::PoseStamped tf_marker; 
        geometry_msgs::msg::PoseArray markers;
        geometry_msgs::msg::PoseArray tf_markers;         
        markers = *msg; 
        // RCLCPP_INFO(this->get_logger(), "Header: %s", marker_pose.header.frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(), "New marker"); 
        if (!markers.poses.empty()){
            for (auto marker : markers.poses){
                marker_pose.header = markers.header;
                marker_pose.header.stamp = this->get_clock()->now();
                marker_pose.pose = marker;
                tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(marker_pose, tf_marker, "link_base",
                    tf2::Duration(std::chrono::seconds(1)));
                tf_markers.poses.push_back(tf_marker.pose);
                // RCLCPP_INFO(this->get_logger(), "Header: %s", tf_marker.header.frame_id.c_str());
                // RCLCPP_INFO(this->get_logger(), "x: %f", tf_marker.pose.position.x); 
                // RCLCPP_INFO(this->get_logger(), "y: %f", tf_marker.pose.position.y);
                // RCLCPP_INFO(this->get_logger(), "z: %f", tf_marker.pose.position.z);
            } 
            tf_markers.header = markers.header;
            tf_markers.header.frame_id = tf_marker.header.frame_id;
            pub->publish(tf_markers);
        };
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformPoseMarker>());
  rclcpp::shutdown();
  return 0;
}