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


using namespace std::chrono_literals;

class Broadcaster : public rclcpp::Node
{
public:
    Broadcaster(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Broadcaster Started");

        // initialize a static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        // timer to publish the transform
        part_broadcast_timer_ = this->create_wall_timer(100ms, std::bind(&Broadcaster::part_broadcast_timer_cb_, this));
        aruco_broadcast_timer_ = this->create_wall_timer(100ms, std::bind(&Broadcaster::aruco_broadcast_timer_cb_, this));



        advanced_camera_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/advanced_logical_camera/image", rclcpp::SensorDataQoS(),
                                                                             std::bind(&Broadcaster::advanced_camera_sub_cb_, this, std::placeholders::_1));

        turtle_camera_subscription_ = this->create_subscription<mage_msgs::msg::Marker>("/aruco_markers", rclcpp::SensorDataQoS(),
                                                                             std::bind(&Broadcaster::turtle_camera_sub_cb_, this, std::placeholders::_1));
        
    }

private:
    // ==================== attributes ====================
    /*!< Boolean parameter to whether or not start the broadcaster */
    bool param_broadcast_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /*!< Static broadcaster object */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    /*!< Broadcaster object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    /*!< Utils object to access utility functions*/
    std::shared_ptr<Utils> utils_ptr_;
    /*!< Wall timer object for the broadcaster*/
    rclcpp::TimerBase::SharedPtr part_broadcast_timer_;
    rclcpp::TimerBase::SharedPtr aruco_broadcast_timer_;

    // Pubs/Subs
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_;
    rclcpp::Subscription<mage_msgs::msg::Marker>::SharedPtr turtle_camera_subscription_;

    // Storage for Marker Position
    std::array<double, 3> aruco_position_;
    geometry_msgs::msg::Quaternion aruco_orientation_;

    // Storage for Part Position
    std::array<double, 3> part_position_;
    geometry_msgs::msg::Quaternion part_orientation_;

    // ==================== methods =======================

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void part_broadcast_timer_cb_();

    void aruco_broadcast_timer_cb_();

    void advanced_camera_sub_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    void turtle_camera_sub_cb_(const mage_msgs::msg::Marker::SharedPtr msg);

}; // Class Broadcaster
