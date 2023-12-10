#include "robot_controller.hpp"

void RWA3::RobotController::cmd_vel_timer_cb()
{
  geometry_msgs::msg::Twist msg;
  if (dist_2_nearest_aruco_ < 0.4)
  {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
  }

  else
  {
    msg.linear.x = 0.1;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
  }
  cmd_vel_publisher_->publish(msg);
}

void RWA3::RobotController::odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_position_.first = msg->pose.pose.position.x;
  robot_position_.second = msg->pose.pose.position.y;
  robot_orientation_.x = msg->pose.pose.orientation.x;
  robot_orientation_.y = msg->pose.pose.orientation.y;
  robot_orientation_.z = msg->pose.pose.orientation.z;
  robot_orientation_.w = msg->pose.pose.orientation.w;
  //RCLCPP_INFO_STREAM(this->get_logger(), "X: " << robot_position_.first << "Y: " << robot_position_.second);
}

void RWA3::RobotController::marker_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  turn_instruction_ = msg->marker_ids[0];
  // RCLCPP_INFO_STREAM(this->get_logger(), "TURN INSTRUCTIONS DETECTED");
  RWA3::RobotController::aruco_frame_listener_();
}

double RWA3::RobotController::calcualte_distance(const std::pair<double, double> &loc1, const std::pair<double, double> &loc2)
{
  double result = sqrt(pow(loc2.first - loc1.first, 2) + pow(loc2.second - loc1.second, 2));
  return result;
}

void RWA3::RobotController::aruco_frame_listener_()
{
  geometry_msgs::msg::TransformStamped aruco;

  try
  {
    aruco = aruco_tf_buffer_->lookupTransform("aruco_marker_frame", "odom", tf2::TimePointZero);
    double aruco_x = aruco.transform.translation.x;
    double aruco_y = aruco.transform.translation.y;

    std::pair<double, double> aruco_position;
    aruco_position.first = aruco_x;
    aruco_position.second = aruco_y;

    dist_2_nearest_aruco_ = calcualte_distance(aruco_position, robot_position_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Distance to Nearest Aruco:" << dist_2_nearest_aruco_);
  }
  catch (const tf2::TransformException &except)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Could not get transform b/w aruco and odom");
  }
  // RCLCPP_INFO_STREAM(this->get_logger(), "Found transform b/w aruco and odom");
}

void RWA3::RobotController::part_frame_listener_()
{
  geometry_msgs::msg::TransformStamped part;

  try
  {
    part = part_tf_buffer_->lookupTransform("part_frame", "odom", tf2::TimePointZero);
  }
  catch (const tf2::TransformException &except)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Could not get transform b/w part and odom");
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Found transform b/w part and odom");
}
