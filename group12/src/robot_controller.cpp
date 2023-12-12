#include "robot_controller.hpp"

double RWA3::RobotController::calculate_distance(const std::pair<double, double> &loc1, const std::pair<double, double> &loc2)
{
  double result = sqrt(pow(loc2.first - loc1.first, 2) + pow(loc2.second - loc1.second, 2));
  return result;
}

void RWA3::RobotController::cmd_vel_timer_cb()
{
  geometry_msgs::msg::Twist msg;

  if (dist_2_nearest_aruco_ <= 1.0)
  {
    RWA3::RobotController::check_turn_instruction();
    if (turn_instruction_ == "right_90")
    {
      if (turn_ctr_ < 33)
      {
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = -0.1;
        turn_ctr_ += 1;
        RCLCPP_INFO_STREAM(this->get_logger(), turn_ctr_);
      }
    }

    else if (turn_instruction_ == "left_90")
    {
      if (turn_ctr_ < 16)
      {
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.1;
        turn_ctr_ += 1;
      }
    }

    else if (turn_instruction_ == "stop")
    {
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
    }
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
}

void RWA3::RobotController::turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  //if (!msg->marker_ids[0])
  {
    marker_id_ = "aruco_marker_" + std::to_string(msg->marker_ids[0]);
    RWA3::RobotController::aruco_broadcast_timer_cb_(msg);
  }
}

void RWA3::RobotController::aruco_broadcast_timer_cb_(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped aruco_transform_stamped;
  aruco_transform_stamped.header.stamp = msg->header.stamp;
  aruco_transform_stamped.header.frame_id = msg->header.frame_id;
  aruco_transform_stamped.child_frame_id = "aruco_marker_frame";
  aruco_transform_stamped.transform.translation.x = msg->poses[0].position.x;
  aruco_transform_stamped.transform.translation.y = msg->poses[0].position.y;
  aruco_transform_stamped.transform.translation.z = msg->poses[0].position.z;
  aruco_transform_stamped.transform.rotation.x = msg->poses[0].orientation.x;
  aruco_transform_stamped.transform.rotation.y = msg->poses[0].orientation.y;
  aruco_transform_stamped.transform.rotation.z = msg->poses[0].orientation.z;
  aruco_transform_stamped.transform.rotation.w = msg->poses[0].orientation.w;

  aruco_tf_broadcaster_->sendTransform(aruco_transform_stamped);
  // RCLCPP_INFO_STREAM(this->get_logger(), "Aruco Broadcaster, X:" << aruco_transform_stamped.transform.translation.x);
}

void RWA3::RobotController::aruco_frame_listener_()
{
  geometry_msgs::msg::TransformStamped aruco;

  try
  {
    aruco = aruco_tf_buffer_->lookupTransform("aruco_marker_frame", "odom", tf2::TimePointZero);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Got Transform b/w aruco and odom");

    double aruco_x = aruco.transform.translation.x;
    double aruco_y = aruco.transform.translation.y;

    std::pair<double, double> aruco_position;
    aruco_position.first = aruco_x;
    aruco_position.second = aruco_y;

    dist_2_nearest_aruco_ = calculate_distance(aruco_position, robot_position_) - 1.5;
    RCLCPP_INFO_STREAM(this->get_logger(), "Aruco X in Odom:" << aruco_x);
    RCLCPP_INFO_STREAM(this->get_logger(), "Distance to Nearest Aruco:" << dist_2_nearest_aruco_);
  }
  catch (const tf2::TransformException &except)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
  }
}

void RWA3::RobotController::advanced_camera_sub_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) // PROBLEM HERE: program dies.
{
  if (!msg)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "LOGICAL CAMERA SUB CB");
    part_color_ = convert_part_color_to_string(msg->part_poses[0].part.type);
    part_type_ = convert_part_type_to_string(msg->part_poses[0].part.color);
    part_position_[0] = msg->part_poses[0].pose.position.x;
    part_position_[1] = msg->part_poses[0].pose.position.y;
    part_position_[2] = msg->part_poses[0].pose.position.z;
    part_orientation_.x = msg->part_poses[0].pose.orientation.x;
    part_orientation_.y = msg->part_poses[0].pose.orientation.y;
    part_orientation_.z = msg->part_poses[0].pose.orientation.z;
    part_orientation_.w = msg->part_poses[0].pose.orientation.w;
  }
  // RWA3::RobotController::part_broadcast_timer_cb_(msg);

  // RWA3::RobotController::part_frame_listener_();
}

void RWA3::RobotController::part_broadcast_timer_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) // PROBLEM HERE: Untested because of previous problem
{
  geometry_msgs::msg::TransformStamped part_transform_stamped;

  part_transform_stamped.header.stamp = this->get_clock()->now();
  part_transform_stamped.header.frame_id = "logical_camera_link";
  part_transform_stamped.child_frame_id = "part_frame";

  part_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
  part_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
  part_transform_stamped.transform.translation.z = msg->part_poses[0].pose.position.z;

  part_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
  part_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
  part_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
  part_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;

  // Send the transform
  RCLCPP_INFO_STREAM(this->get_logger(), "LOGICAL CAMERA BROADCASTER CB");
  part_tf_broadcaster_->sendTransform(part_transform_stamped);
}

void RWA3::RobotController::part_frame_listener_() // PROBLEM HERE: untested because of prvious problem.
{
  geometry_msgs::msg::TransformStamped part;

  try
  {
    part = aruco_tf_buffer_->lookupTransform("part_frame", "odom", tf2::TimePointZero);
    RCLCPP_INFO_STREAM(this->get_logger(), "Found transform b/w part and odom");
  }
  catch (const tf2::TransformException &except)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
  }
}

void RWA3::RobotController::check_turn_instruction()
{
  if (this->has_parameter(marker_id_))
  {
    turn_instruction_ = this->get_parameter(marker_id_).get_value<std::string>();
  }
}

std::string RWA3::RobotController::convert_part_type_to_string(unsigned int part_type)
{
  if (part_type == mage_msgs::msg::Part::BATTERY)
    return "battery";
  else if (part_type == mage_msgs::msg::Part::PUMP)
    return "pump";
  else if (part_type == mage_msgs::msg::Part::REGULATOR)
    return "regulator";
  else if (part_type == mage_msgs::msg::Part::SENSOR)
    return "sensor";
  else
    return "unknown";
}

std::string RWA3::RobotController::convert_part_color_to_string(unsigned int part_color)
{
  if (part_color == mage_msgs::msg::Part::RED)
    return "red";
  else if (part_color == mage_msgs::msg::Part::GREEN)
    return "green";
  else if (part_color == mage_msgs::msg::Part::BLUE)
    return "blue";
  else if (part_color == mage_msgs::msg::Part::PURPLE)
    return "purple";
  else if (part_color == mage_msgs::msg::Part::ORANGE)
    return "orange";
  else
    return "unknown";
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RWA3::RobotController>("robot_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
