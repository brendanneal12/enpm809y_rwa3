#include "robot_controller.hpp"
#include <geometry_msgs/msg/twist.hpp>

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




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RWA3::RobotController>("robot_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
}