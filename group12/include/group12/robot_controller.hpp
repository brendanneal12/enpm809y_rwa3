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
#include <nav_msgs/msg/odometry.hpp>
/**
 * @brief Namspace used for RWA3
 *
 */

namespace RWA3
{
    /**
     * @brief Robot Controller Node
     *
     */
    class RobotController : public rclcpp::Node
    {
    public:
        RobotController(std::string node_name) : Node(node_name)
        {
            // Declare parameters from .yaml file.
            this->declare_parameter("aruco_marker_0", "right_90");
            aruco_marker_0_ = this->get_parameter("aruco_marker_0").as_string();

            this->declare_parameter("aruco_marker_1", "left_90");
            aruco_marker_1_ = this->get_parameter("aruco_marker_1").as_string();

            this->declare_parameter("aruco_marker_2", "end");
            aruco_marker_2_ = this->get_parameter("aruco_marker_2").as_string();

            // Load a buffer of transforms
            aruco_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            aruco_tf_buffer_->setUsingDedicatedThread(true);

            part_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            part_tf_buffer_->setUsingDedicatedThread(true);

            aruco_tf_listener = std::make_shared<tf2_ros::TransformListener>(*aruco_tf_buffer_);
            part_tf_listener = std::make_shared<tf2_ros::TransformListener>(*part_tf_buffer_);

            // Set up command velocity publisher and bind it to a timer callback.
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            cmd_vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotController::cmd_vel_timer_cb, this));

            // Set up odometry subscription and bind it to a callback.
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotController::odom_sub_cb_, this, std::placeholders::_1));

            // Set up marker subscriptio  and bind it to a callback.
            marker_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", rclcpp::SensorDataQoS(),
                                                                                                       std::bind(&RobotController::marker_sub_cb_, this, std::placeholders::_1));
        }

    private:
        // ==================== parameters ====================
        std::string aruco_marker_0_;
        std::string aruco_marker_1_;
        std::string aruco_marker_2_;

        // ==================== attributes ====================
        // Pubs/Subs
        rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_subscription_;

        std::shared_ptr<tf2_ros::TransformListener> aruco_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> aruco_tf_buffer_;

        std::shared_ptr<tf2_ros::TransformListener> part_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> part_tf_buffer_;

        // Robot Attributes
        std::pair<double,double> robot_position_;
        geometry_msgs::msg::Quaternion robot_orientation_;
        int turn_instruction_;
        double dist_2_nearest_aruco_;

        // ==================== methods =======================

        /**
         * @brief Timer to publish cmd_vel messages
         *
         */
        void cmd_vel_timer_cb();

        /**
         * @brief Subscriber callback to update current position of turtlebot.
         * @param msg
         */

        void odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
         * @brief Subscriber callback to update current "turn instruction" of turtlebot.
         * @param msg
         */

        void marker_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        double calcualte_distance(const std::pair<double, double> &loc1, const std::pair<double, double> &loc2);

        void part_frame_listener_();
        void aruco_frame_listener_();

    }; // Class Robot Controller
} // Namespace RWA3