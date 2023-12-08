#include "robot_controller.hpp"

void RWA3::RobotController::cmd_vel_timer_cb()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.1;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  cmd_vel_publisher_->publish(msg);
}

void RWA3::RobotController::odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_position_[0] = msg->pose.pose.position.x;
  robot_position_[1] = msg->pose.pose.position.y;
  robot_position_[2] = msg->pose.pose.position.z;
  // RCLCPP_INFO_STREAM(this->get_logger(), "X: " << position_.first << "Y: " << position_.second);
}

void RWA3::RobotController::marker_subscription_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  (void) msg; //TODO: HERE
  // turn_instruction_ = msg->marker_id
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RWA3::RobotController>("robot_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
}