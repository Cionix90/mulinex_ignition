#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


class GTOdomPublisher : public rclcpp::Node
{
public:
  GTOdomPublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    
    std::string topic_name = "gt_odom";

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_name, 10,
      std::bind(&GTOdomPublisher::broadcast_tf2, this, std::placeholders::_1));
  }

private:
  void broadcast_tf2(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // // Read message content and assign it to
    // // corresponding tf variables
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "gt_odom";
    t.child_frame_id = "base_link";

    // // Turtle only exists in 2D, thus we get x and y translation
    // // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    // // For the same reason, turtle can only rotate around one axis
    // // and this why we set rotation in x and y to 0 and obtain
    // // rotation in z axis from the message
    
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GTOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}