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

void RWA3::RobotController::odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg) {
  position_.first = msg->pose.pose.position.x;
  position_.second=msg->pose.pose.position.y;
  //RCLCPP_INFO_STREAM(this->get_logger(), "X: " << position_.first << "Y: " << position_.second);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RWA3::RobotController>("robot_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
}