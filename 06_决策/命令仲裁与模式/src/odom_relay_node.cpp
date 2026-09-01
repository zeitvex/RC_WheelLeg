#include "sim2real_runtime/odom_relay_node.hpp"

namespace sim2real_runtime
{

OdomRelayNode::OdomRelayNode()
: Node("odom_relay_node")
{
  odom_input_topic_ = declare_parameter<std::string>("odom_input_topic", "/odin1/odometry");
  odom_output_topic_ = declare_parameter<std::string>("odom_output_topic", "/odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_input_topic_, 10,
    std::bind(&OdomRelayNode::onOdom, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic_, 10);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  RCLCPP_INFO(get_logger(),
    "Odom relay: %s -> %s (base_frame=%s, publish_tf=%s)",
    odom_input_topic_.c_str(), odom_output_topic_.c_str(),
    base_frame_.c_str(), publish_tf_ ? "true" : "false");
}

void OdomRelayNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Remap child_frame_id and republish
  auto out_msg = *msg;
  out_msg.header.frame_id = "odom";
  out_msg.child_frame_id = base_frame_;
  odom_pub_->publish(out_msg);

  // Broadcast TF: odom → base_link
  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }
}

}  // namespace sim2real_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sim2real_runtime::OdomRelayNode>());
  rclcpp::shutdown();
  return 0;
}
