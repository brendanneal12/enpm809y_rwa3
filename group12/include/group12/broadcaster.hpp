#pragma once
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <mage_msgs/msg/marker.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include "robot_controller.hpp"
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class Broadcaster : public rclcpp::Node
{
public:
    Broadcaster(std::shared_ptr<RWA3::RobotController> const &robot_controller) : Node("broadcaster")
    {

        n_robot_controller = robot_controller;
        RCLCPP_INFO(this->get_logger(), "Broadcaster Started");

        // initialize the transform broadcaster
        aruco_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        part_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        // this->advanced_camera_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/advanced_logical_camera/image", rclcpp::SensorDataQoS(),
        //                                                                                                       std::bind(&Broadcaster::advanced_camera_sub_cb_, this, std::placeholders::_1));

        this->turtle_camera_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", rclcpp::SensorDataQoS(),
                                                                                                                std::bind(&Broadcaster::turtle_camera_sub_cb_, this, std::placeholders::_1));
    }

private:
    // ==================== attributes ====================
    std::shared_ptr<RWA3::RobotController> n_robot_controller;
    std::shared_ptr<Utils> utils_ptr_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> aruco_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr aruco_broadcast_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr part_broadcast_timer_;

    // Pubs/Subs
    // rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr turtle_camera_subscription_;

    // Storage for Marker Position
    std::array<double, 3> aruco_position_;
    geometry_msgs::msg::Quaternion aruco_orientation_;

    // Storage for Part Position
    std::string part_type_;
    std::string part_color_;
    std::array<double, 3> part_position_;
    geometry_msgs::msg::Quaternion part_orientation_;

    // ==================== methods =======================
    /**
     * @brief Convert a part type to a string
     *
     * @param unsinged int part_type
     * @return battery
     * @return regulator
     * @return sensor
     * @return pump
     * @return unknown
     */
    std::string convert_part_type_to_string(unsigned int part_type);

    /**
     * @brief Convert a part color to a string
     *
     * @param unsinged int part_color
     * @return red
     * @return green
     * @return blue
     * @return purple
     * @return orange
     * @return unknown
     */
    std::string convert_part_color_to_string(unsigned int part_color);

    // void part_broadcast_timer_cb_();
    // void advanced_camera_sub_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    void aruco_broadcast_timer_cb_();
    void turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);


}; // Class Broadcaster
