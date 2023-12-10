#include <broadcaster.hpp>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

// void Broadcaster::advanced_camera_sub_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
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

void Broadcaster::turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    if (!msg->marker_ids[0])
    {
        aruco_position_[0] = msg->poses[0].position.x;
        aruco_position_[1] = msg->poses[0].position.y;
        aruco_position_[2] = msg->poses[0].position.z;
        aruco_orientation_.x = msg->poses[0].orientation.x;
        aruco_orientation_.y = msg->poses[0].orientation.y;
        aruco_orientation_.z = msg->poses[0].orientation.z;
        aruco_orientation_.w = msg->poses[0].orientation.w;

        Broadcaster::aruco_broadcast_timer_cb_();

    }
}

// void Broadcaster::part_broadcast_timer_cb_()
// {
//     geometry_msgs::msg::TransformStamped part_transform_stamped;

//     part_transform_stamped.header.stamp = this->get_clock()->now();
//     part_transform_stamped.header.frame_id = "logical_camera_link";
//     part_transform_stamped.child_frame_id = "part_frame";

//     part_transform_stamped.transform.translation.x = part_position_[0];
//     part_transform_stamped.transform.translation.y = part_position_[1];
//     part_transform_stamped.transform.translation.z = part_position_[2];

//     part_transform_stamped.transform.rotation.x = part_orientation_.x;
//     part_transform_stamped.transform.rotation.y = part_orientation_.y;
//     part_transform_stamped.transform.rotation.z = part_orientation_.z;
//     part_transform_stamped.transform.rotation.w = part_orientation_.w;

//     // Send the transform
//     part_tf_broadcaster_->sendTransform(part_transform_stamped);
// }

void Broadcaster::aruco_broadcast_timer_cb_()
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
}

std::string Broadcaster::convert_part_type_to_string(unsigned int part_type)
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

std::string Broadcaster::convert_part_color_to_string(unsigned int part_color)
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
    auto robot_controller = std::make_shared<RWA3::RobotController>("robot_controller");
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<Broadcaster>(robot_controller);
    exec.add_node(node);
    exec.add_node(robot_controller);
    exec.spin();
    rclcpp::shutdown();
}
