#pragma once
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
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
            this->declare_parameter("aruco_marker_0", "right_90");
            aruco_marker_0_ = this->get_parameter("aruco_marker_0").as_string();

            this->declare_parameter("aruco_marker_1", "left_90");
            aruco_marker_1_ = this->get_parameter("aruco_marker_1").as_string();

            this->declare_parameter("aruco_marker_2", "end");
            aruco_marker_2_ = this->get_parameter("aruco_marker_2").as_string();

            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            cmd_vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotController::cmd_vel_timer_cb, this));

            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotController::odom_sub_cb_, this, std::placeholders::_1));

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

        //Robot Attributes
        std::pair<double, double> position_;

        // ==================== methods =======================

        /**
         * @brief Timer to publish cmd_vel messages
         *
         */
        void cmd_vel_timer_cb();

        void odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg);

    }; // Class Robot Controller
} // Namespace RWA3