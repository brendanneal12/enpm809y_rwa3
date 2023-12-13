#include "robot_controller.hpp"

double RWA3::RobotController::calculate_distance(const std::array<double, 3> &loc1, const std::array<double, 3> &loc2)
{
  // Calculate Euclidian X-Y distance between two objects.
  double result = sqrt(pow(loc2[0] - loc1[0], 2) + pow(loc2[1] - loc1[1], 2));
  return result;
}

void RWA3::RobotController::cmd_vel_timer_cb()
{
  // Initialize Message
  geometry_msgs::msg::Twist msg;
  // Check current turn instruction
  RWA3::RobotController::check_turn_instruction();
  // If at the finish line
  if (at_finish_)
  {
    // Stop
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    // Print detected Parts
    RWA3::RobotController::print_seen_parts();
  }

  // If not at finish
  if (!at_finish_)
  {
    // If the instruction is to turn right
    if (turn_instruction_ == "right_90")
    {
      // And if the robot is at the required distance
      if (dist_2_nearest_aruco_ <= 0.825)
      {
        // Execute a right 90 degree turn
        in_turn_ = true;
        if (turn_ctr_ < 34)
        {
          msg.linear.x = 0.0;
          msg.linear.y = 0.0;
          msg.linear.z = 0.0;
          msg.angular.x = 0.0;
          msg.angular.y = 0.0;
          msg.angular.z = -0.1;
          turn_ctr_ += 1;
          RCLCPP_INFO_STREAM(this->get_logger(), "Executing Right 90 Turn");
        }
        else
        {
          // Once turn is complete, set in_turn_ flag to false.
          in_turn_ = false;
        }
      }
      // If the robot is not at the requried distance
      else
      {
        // Drive straight
        turn_ctr_ = 0;
        msg.linear.x = 0.1;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
      }
    }
    // If the turn instruction is to turn left
    else if (turn_instruction_ == "left_90")
    {
      // And if the robot is at the required distance
      if (dist_2_nearest_aruco_ <= 0.825)
      {
        // Execute a left 90 degree turn.
        in_turn_ = true;
        if (turn_ctr_ < 34)
        {
          msg.linear.x = 0.0;
          msg.linear.y = 0.0;
          msg.linear.z = 0.0;
          msg.angular.x = 0.0;
          msg.angular.y = 0.0;
          msg.angular.z = 0.1;
          turn_ctr_ += 1;
          RCLCPP_INFO_STREAM(this->get_logger(), "Executing Left 90 Turn");
        }
        else
        {
          // Once turn is complete, set in_turn_ flag to false.
          in_turn_ = false;
        }
      }
      // If the robot is not at the requried distance
      else
      {
        // Drive straight
        turn_ctr_ = 0;
        msg.linear.x = 0.1;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
      }
    }
    // If the turn instruction is to stop
    else if (turn_instruction_ == "end")
    {
      // And if the robot is at the required distance
      if (dist_2_nearest_aruco_ <= 0.735)
      {
        // Robot has reached goal
        at_finish_ = true;
      }
      // If not
      else
      {
        // Drive straight
        msg.linear.x = 0.1;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
      }
    }
  }
  // Publish message determined by navigation logic.
  cmd_vel_publisher_->publish(msg);
}

void RWA3::RobotController::odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Update robot's current position and orientation.
  robot_position_[0] = msg->pose.pose.position.x;
  robot_position_[1] = msg->pose.pose.position.y;
  robot_position_[2] = msg->pose.pose.position.z;
  robot_orientation_.x = msg->pose.pose.orientation.x;
  robot_orientation_.y = msg->pose.pose.orientation.y;
  robot_orientation_.z = msg->pose.pose.orientation.z;
  robot_orientation_.w = msg->pose.pose.orientation.w;
}

void RWA3::RobotController::turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  // If not at finish
  if (!at_finish_)
  {
    // If not currently taking a turn
    if (!in_turn_)
    {
      // Look for and broadcast detected aruco marker position.
      marker_id_ = "aruco_marker_" + std::to_string(msg->marker_ids[0]);
      RWA3::RobotController::aruco_broadcast_timer_cb_(msg);
    }
  }
}

void RWA3::RobotController::aruco_broadcast_timer_cb_(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  // Broadcast aruco marker position to tf tree.
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
}

void RWA3::RobotController::aruco_frame_listener_()
{
  geometry_msgs::msg::TransformStamped aruco;

  try
  {
    // Look up transform between aruco marker and odom and store.
    aruco = aruco_tf_buffer_->lookupTransform("odom", "aruco_marker_frame", tf2::TimePointZero);
    double aruco_x = aruco.transform.translation.x;
    double aruco_y = aruco.transform.translation.y;
    double aruco_z = aruco.transform.translation.z;

    // If not at finish
    if (!at_finish_)
    {
      // If not currently taking a turn
      if (!in_turn_)
      {
        // Update distance to nearest detected aruco marker.
        std::array<double, 3> aruco_position;
        aruco_position[0] = aruco_x;
        aruco_position[1] = aruco_y;
        aruco_position[2] = aruco_z;

        dist_2_nearest_aruco_ = calculate_distance(aruco_position, robot_position_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Distance to detected Aruco:" << dist_2_nearest_aruco_);
      }
    }
  }
  catch (const tf2::TransformException &except)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
  }
}

void RWA3::RobotController::advanced_camera_sub_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // If the incoming message is not empty.
  if (msg->part_poses.size() != 0)
  {
    // Convert and store part type and color
    part_type_ = RWA3::RobotController::convert_part_type_to_string(msg->part_poses[0].part.type);
    part_color_ = RWA3::RobotController::convert_part_color_to_string(msg->part_poses[0].part.color);

    // Broadcast part position to tf tree.
    RWA3::RobotController::part_broadcast_timer_cb_(msg);
  }
}

void RWA3::RobotController::part_broadcast_timer_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
  // Broadcast part position to tf tree.
  geometry_msgs::msg::TransformStamped part_transform_stamped;
  part_transform_stamped.header.stamp = current_time_;
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
  part_tf_broadcaster_->sendTransform(part_transform_stamped);
}

void RWA3::RobotController::part_frame_listener_()
{
  geometry_msgs::msg::TransformStamped part;

  try
  {
    // Look up transformation between detected part and odom frame and store.
    part = aruco_tf_buffer_->lookupTransform("odom", "part_frame", tf2::TimePointZero);
    std::array<double, 3> part_location;
    part_location[0] = part.transform.translation.x;
    part_location[1] = part.transform.translation.y;
    part_location[2] = part.transform.translation.z;
    geometry_msgs::msg::Quaternion part_rotation;
    part_rotation.x = part.transform.rotation.x;
    part_rotation.y = part.transform.rotation.y;
    part_rotation.z = part.transform.rotation.z;
    part_rotation.w = part.transform.rotation.w;
    // Add detected part to custom data structure.
    RWA3::RobotController::add_seen_part(part_color_, part_type_, part_location, part_rotation);
  }
  catch (const tf2::TransformException &except)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), except.what());
  }
}

void RWA3::RobotController::clock_sub_cb_(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  // Update current time attribute. Needed for broadcaster to work.
  current_time_ = msg->clock;
}

void RWA3::RobotController::check_turn_instruction()
{
  // Check parameter list and compare to marker id.
  // If the marker ID matches a current parameter.
  if (this->has_parameter(marker_id_))
  {
    // Get the paremetr's associated turn instruction.
    turn_instruction_ = this->get_parameter(marker_id_).get_value<std::string>();
  }
}

bool RWA3::RobotController::check_duplicate_parts(const std::string &new_part_color, const std::string &new_part_type)
{
  // For all parts (rows) in custom data structure
  for (const auto &part : detected_parts_)
  {
    // Compare new part color and type to existing members.
    if (std::get<0>(part) == new_part_color && std::get<1>(part) == new_part_type)
    {
      // Return true if there is a match
      return true;
    }
  }
  // False otherwise.
  return false;
}

void RWA3::RobotController::add_seen_part(const std::string &color, const std::string &type, const std::array<double, 3> &position, const geometry_msgs::msg::Quaternion &orientation)
{
  // Check if new detected part matches an existing member.
  bool duplicate_check = RWA3::RobotController::check_duplicate_parts(color, type);
  // If it is unique
  if (!duplicate_check)
  {
    // Add new part to data structure.
    detected_parts_.emplace_back(color, type, position, orientation);
    RCLCPP_INFO_STREAM(this->get_logger(), "Detected a Part: " << color << " " << type);
  }
}

void RWA3::RobotController::print_seen_parts()
{
  // For all parts in data structure.
  for (const auto &part : detected_parts_)
  {
    // Convert the orientation from geometry_msg quaternion to tf2_quaternion
    tf2::Quaternion tf2_quaternion;
    tf2::convert(std::get<3>(part), tf2_quaternion);
    // Convert to Euler angles.
    auto rpy = utils_ptr_->set_euler_from_quaternion(tf2_quaternion);

    // Print
    RCLCPP_INFO_STREAM(this->get_logger(), std::get<0>(part) << " " << std::get<1>(part) << " detected at xyz=[" << std::get<2>(part)[0] << " " << std::get<2>(part)[1] << " " << std::get<2>(part)[2] << "] rpy=["
                                                             << rpy.at(0) << " " << rpy.at(1) << " " << rpy.at(2) << "]\n");
  }
}
std::string RWA3::RobotController::convert_part_type_to_string(uint8_t part_type)
{
  // Match numeric part type to string part type based off of message definition.
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

std::string RWA3::RobotController::convert_part_color_to_string(uint8_t part_color)
{
  // Match numeric part color to string part color based off of message definition.
  if (part_color == mage_msgs::msg::Part::RED)
    return "Red";
  else if (part_color == mage_msgs::msg::Part::GREEN)
    return "Green";
  else if (part_color == mage_msgs::msg::Part::BLUE)
    return "Blue";
  else if (part_color == mage_msgs::msg::Part::PURPLE)
    return "Purple";
  else if (part_color == mage_msgs::msg::Part::ORANGE)
    return "Orange";
  else
    return "Uknown";
}

rcl_interfaces::msg::SetParametersResult RWA3::RobotController::parameters_cb(const std::vector<rclcpp::Parameter> &parameters)
{
  // Watch for change in parameters and set the result.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters)
  {
    if (param.get_name() == "aruco_marker_0")
    {
      aruco_marker_0_ = param.as_string();
    }
    else if (param.get_name() == "aruco_marker_1")
    {
      aruco_marker_1_ = param.as_string();
    }
    else if (param.get_name() == "aruco_marker_2")
    {
      aruco_marker_2_ = param.as_string();
    }
    else
    {
      result.successful = false;
      result.reason = "parameter not authorized to be modified";
    }
  }
  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RWA3::RobotController>("robot_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
