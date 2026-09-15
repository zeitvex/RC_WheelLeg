#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace sim2real_runtime
{

/// Subscribes to odin_ros_driver's odometry (e.g. /odin1/odometry),
/// remaps child_frame_id to "base_link", republishes on /odom,
/// and broadcasts the odom → base_link TF.
class OdomRelayNode : public rclcpp::Node
{
public:
  OdomRelayNode();

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_input_topic_;
  std::string odom_output_topic_;
  std::string base_frame_;
  bool publish_tf_;
};

}  // namespace sim2real_runtime
