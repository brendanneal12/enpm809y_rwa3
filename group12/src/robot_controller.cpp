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
  robot_orientation_.x = msg->pose.pose.orientation.x;
  robot_orientation_.y = msg->pose.pose.orientation.y;
  robot_orientation_.z = msg->pose.pose.orientation.z;
  robot_orientation_.w = msg->pose.pose.orientation.w;
  // RCLCPP_INFO_STREAM(this->get_logger(), "X: " << position_.first << "Y: " << position_.second);
}

void RWA3::RobotController::marker_sub_cb_(const mage_msgs::msg::Marker::SharedPtr msg)
{
  turn_instruction_ =  msg->id;
  RCLCPP_INFO_STREAM(this->get_logger(), "TURN INSTRUCTIONS DETECTED");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RWA3::RobotController>("robot_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
}