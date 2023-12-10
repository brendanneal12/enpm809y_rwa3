#include "robot_controller.hpp"

double RWA3::RobotController::calcualte_distance(const std::pair<double, double> &loc1, const std::pair<double, double> &loc2)
{
  double result = sqrt(pow(loc2.first - loc1.first, 2) + pow(loc2.second - loc1.second, 2));
  return result;
}

void RWA3::RobotController::cmd_vel_timer_cb()
{
  geometry_msgs::msg::Twist msg;

  if (dist_2_nearest_aruco_ <= 0.4)
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
}

void RWA3::RobotController::turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  if (!msg->marker_ids[0])
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "RGB Camera Sub Callback");
    aruco_position_[0] = msg->poses[0].position.x;
    aruco_position_[1] = msg->poses[0].position.y;
    aruco_position_[2] = msg->poses[0].position.z;
    aruco_orientation_.x = msg->poses[0].orientation.x;
    aruco_orientation_.y = msg->poses[0].orientation.y;
    aruco_orientation_.z = msg->poses[0].orientation.z;
    aruco_orientation_.w = msg->poses[0].orientation.w;

    for (int i = 0; i < 10; i++)
    {
      RWA3::RobotController::aruco_broadcast_timer_cb_();
    }

    RWA3::RobotController::aruco_frame_listener_();
  }
}

void RWA3::RobotController::aruco_broadcast_timer_cb_()
{
  geometry_msgs::msg::TransformStamped aruco_transform_stamped;
  aruco_transform_stamped.header.stamp = this->get_clock()->now();
  aruco_transform_stamped.header.frame_id = "camera_rgb_optical_frame";
  aruco_transform_stamped.child_frame_id = "aruco_marker_frame";

  aruco_transform_stamped.transform.translation.x = aruco_position_[0];
  aruco_transform_stamped.transform.translation.y = aruco_position_[1];
  aruco_transform_stamped.transform.translation.z = aruco_position_[2];

  aruco_transform_stamped.transform.rotation.x = aruco_orientation_.x;
  aruco_transform_stamped.transform.rotation.y = aruco_orientation_.y;
  aruco_transform_stamped.transform.rotation.z = aruco_orientation_.z;
  aruco_transform_stamped.transform.rotation.w = aruco_orientation_.w;

  aruco_tf_broadcaster_->sendTransform(aruco_transform_stamped);
  RCLCPP_INFO_STREAM(this->get_logger(), "Aruco Broadcaster");
}

void RWA3::RobotController::aruco_frame_listener_()
{
  geometry_msgs::msg::TransformStamped aruco;

  try
  {
    aruco = aruco_tf_buffer_->lookupTransform("aruco_marker_frame", "odom", tf2::TimePointZero);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got Transform b/w aruco and odom");

    // double aruco_x = aruco.transform.translation.x;
    // // RCLCPP_INFO_STREAM(this->get_logger(), "Aruco X in Odom:" << aruco_x);
    // double aruco_y = aruco.transform.translation.y;

    // std::pair<double, double> aruco_position;
    // aruco_position.first = aruco_x;
    // aruco_position.second = aruco_y;

    // dist_2_nearest_aruco_ = calcualte_distance(aruco_position, robot_position_);
    // // RCLCPP_INFO_STREAM(this->get_logger(), "Distance to Nearest Aruco:" << dist_2_nearest_aruco_);

  }
  catch (const tf2::TransformException &except)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Could not get transform b/w aruco and odom");
  }
}

// void RWA3::RobotController::advanced_camera_sub_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
// {
//     if (!msg)
//     {
//         RCLCPP_INFO_STREAM(this->get_logger(), "LOGICAL CAMERA SUB CB");
//         part_color_ = convert_part_color_to_string(msg->part_poses[0].part.type);
//         part_type_ = convert_part_type_to_string(msg->part_poses[0].part.color);
//         part_position_[0] = msg->part_poses[0].pose.position.x;
//         part_position_[1] = msg->part_poses[0].pose.position.y;
//         part_position_[2] = msg->part_poses[0].pose.position.z;
//         part_orientation_.x = msg->part_poses[0].pose.orientation.x;
//         part_orientation_.y = msg->part_poses[0].pose.orientation.y;
//         part_orientation_.z = msg->part_poses[0].pose.orientation.z;
//         part_orientation_.w = msg->part_poses[0].pose.orientation.w;

//         Broadcaster::part_broadcast_timer_cb_();

//         Broadcaster::part_frame_listener_();
//     }
// }

// void RWA3::RobotController::part_broadcast_timer_cb_()
// {
//   geometry_msgs::msg::TransformStamped part_transform_stamped;

//   part_transform_stamped.header.stamp = this->get_clock()->now();
//   part_transform_stamped.header.frame_id = "logical_camera_link";
//   part_transform_stamped.child_frame_id = "part_frame";

//   part_transform_stamped.transform.translation.x = part_position_[0];
//   part_transform_stamped.transform.translation.y = part_position_[1];
//   part_transform_stamped.transform.translation.z = part_position_[2];

//   part_transform_stamped.transform.rotation.x = part_orientation_.x;
//   part_transform_stamped.transform.rotation.y = part_orientation_.y;
//   part_transform_stamped.transform.rotation.z = part_orientation_.z;
//   part_transform_stamped.transform.rotation.w = part_orientation_.w;

//   // Send the transform
//   part_tf_broadcaster_->sendTransform(part_transform_stamped);
// }

// void RWA3::RobotController::aruco_frame_listener_()
// {
//   geometry_msgs::msg::TransformStamped aruco;

//   try
//   {
//     aruco = aruco_tf_buffer_->lookupTransform("aruco_marker_frame", "odom", tf2::TimePointZero);
//   }
//   catch (const tf2::TransformException &except)
//   {
//     // RCLCPP_INFO_STREAM(this->get_logger(), "Could not get transform b/w aruco and odom");
//   }
//   // RCLCPP_INFO_STREAM(this->get_logger(), "Found transform b/w aruco and odom");
// }

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
