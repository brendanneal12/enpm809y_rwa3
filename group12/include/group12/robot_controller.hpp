#pragma once
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mage_msgs/msg/marker.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

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

            // Set up command velocity publisher and bind it to a timer callback.
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            cmd_vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotController::cmd_vel_timer_cb, this));

            // Set up odometry subscription and bind it to a callback.
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotController::odom_sub_cb_, this, std::placeholders::_1));


            // Set up marker subscriptio  and bind it to a callback.
            marker_subscription_ = this->create_subscription<mage_msgs::msg::Marker>("/aruco_markers", rclcpp::SensorDataQoS(),
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
        rclcpp::Subscription<mage_msgs::msg::Marker>::SharedPtr marker_subscription_;

        // Robot Attributes
        std::array<double, 3> robot_position_;
        geometry_msgs::msg::Quaternion robot_orientation_;
        int turn_instruction_;

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

        void marker_sub_cb_(const mage_msgs::msg::Marker::SharedPtr msg);

    }; // Class Robot Controller
} // Namespace RWA3