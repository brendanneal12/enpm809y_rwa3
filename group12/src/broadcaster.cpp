#include <broadcaster.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utils.hpp>
// needed for the listener
#include <tf2/exceptions.h>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

void Broadcaster::broadcast_timer_cb_()
{
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;

    // RCLCPP_INFO(this->get_logger(), "Broadcasting dynamic_frame");
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "odom";
    dynamic_transform_stamped.child_frame_id = "dynamic_frame";

    dynamic_transform_stamped.transform.translation.x = 5.0;
    dynamic_transform_stamped.transform.translation.y = 0.4;
    dynamic_transform_stamped.transform.translation.z = 0.3;

    geometry_msgs::msg::Quaternion quaternion = utils_ptr_->set_quaternion_from_euler(M_PI, M_PI / 2, M_PI / 3);
    dynamic_transform_stamped.transform.rotation.x = quaternion.x;
    dynamic_transform_stamped.transform.rotation.y = quaternion.y;
    dynamic_transform_stamped.transform.rotation.z = quaternion.z;
    dynamic_transform_stamped.transform.rotation.w = quaternion.w;
    // Send the transform
    tf_broadcaster_->sendTransform(dynamic_transform_stamped);
}

void Broadcaster::marker_subscription_cb_(const mage_msgs::msg::Marker::SharedPtr msg)
{
    // aruco_position_[0] = msg->pose.position.x
    // aruco_position_[1] = msg->
    // aruco_position_[2] = msg->
    // aruco_orientation_.w = msg->
    // aruco_orientation_.x = msg->
    // aruco_orientation_.y = msg->
    // aruco_orientation_.z = msg->
}

void Broadcaster::part_subscription_cb_(const mage_msgs::msg::Part::SharedPtr msg)
{
    // part_position_[0] = msg->
    // part_position_[1] = msg->
    // part_position_[2] = msg->
    // part_orientation_.w = msg->
    // part_orientation_.x = msg->
    // part_orientation_.y = msg->
    // part_orientation_.z = msg->
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Broadcaster>("broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
}