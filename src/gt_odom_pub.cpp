#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


class GTOdomPublisher : public rclcpp::Node
{
public:
  GTOdomPublisher()
  : Node("ignition_ground_truth")
  {
    std::string robot_name;
    // Declare and acquire `turtlename` parameter
    this->declare_parameter<std::string>("robot_name", "omnicar");
    robot_name = this->get_parameter("robot_name").as_string();
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    
    base_name_ = robot_name + "base_link";
    odom_name_ = robot_name + "gt_odom";
    path_msg_.poses.resize(0); 

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
     robot_name + "gt_odom", 10,
      std::bind(&GTOdomPublisher::broadcast_tf2, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        robot_name + "gt_pose", 10
      );
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
       robot_name + "gt_path", 10
      );
  }

private:
  void broadcast_tf2(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::PoseStamped p;

    // // Read message content and assign it to
    // // corresponding tf variables
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = odom_name_;
    t.child_frame_id = base_name_;
    p.header.stamp = msg->header.stamp;
    p.header.frame_id = odom_name_;
    path_msg_.header.stamp = msg->header.stamp;
    path_msg_.header.frame_id = odom_name_;


    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    

    
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;
    p.set__pose(msg->pose.pose);
    path_msg_.poses.push_back(p);
    if(path_msg_.poses.size() > max_poses_)
      path_msg_.poses.erase(path_msg_.poses.begin());
    tf_broadcaster_->sendTransform(t);
    pose_pub_->publish(p);
    path_pub_->publish(path_msg_);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string robot_name_, base_name_, odom_name_;
  nav_msgs::msg::Path path_msg_;
  long unsigned int max_poses_ = 1000;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GTOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}