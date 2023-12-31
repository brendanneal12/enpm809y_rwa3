#pragma once
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <tf2/exceptions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosgraph_msgs/msg/clock.hpp>
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
        /**
         * @brief Constructor
         * @param node_name
        */
        RobotController(std::string node_name) : Node(node_name)
        {
            // Declare parameters -> Grabs from .yaml
            this->declare_parameter("aruco_marker_0", "right_90");
            aruco_marker_0_ = this->get_parameter("aruco_marker_0").as_string();

            this->declare_parameter("aruco_marker_1", "left_90");
            aruco_marker_1_ = this->get_parameter("aruco_marker_1").as_string();

            this->declare_parameter("aruco_marker_2", "end");
            aruco_marker_2_ = this->get_parameter("aruco_marker_2").as_string();

            // Initialize the transform broadcasters
            aruco_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            part_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

            // Create a utils object to use the utility functions
            utils_ptr_ = std::make_shared<Utils>();

            // Load buffers of transforms and associated listeners

            // Aruco
            aruco_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            aruco_tf_buffer_->setUsingDedicatedThread(true);
            aruco_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*aruco_tf_buffer_);

            // Parts
            part_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            part_tf_buffer_->setUsingDedicatedThread(true);
            part_tf_listener = std::make_shared<tf2_ros::TransformListener>(*part_tf_buffer_);

            // Set up command velocity publisher and bind it to a timer callback.
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            cmd_vel_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotController::cmd_vel_timer_cb, this));

            // Set up odometry subscription and bind it to a callback.
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotController::odom_sub_cb_, this, std::placeholders::_1));

            // Set up marker subscription and bind it to a callback.
            turtle_camera_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", rclcpp::SensorDataQoS(),
                                                                                                              std::bind(&RobotController::turtle_camera_sub_cb_, this, std::placeholders::_1));

            // Set up part subscription and bind it to a callback.
            advanced_camera_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/advanced_logical_camera/image", rclcpp::SensorDataQoS(),
                                                                                                                  std::bind(&RobotController::advanced_camera_sub_cb_, this, std::placeholders::_1));

            // Set up clock subscriptions and bind it to a callback.
            clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SensorDataQoS(),
                                                                                       std::bind(&RobotController::clock_sub_cb_, this, std::placeholders::_1));
            // Set up Listener Timers.
            aruco_listener_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotController::aruco_frame_listener_, this));

            part_listener_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotController::part_frame_listener_, this));

            // Add a parameter callback.
            parameter_cb_ = this->add_on_set_parameters_callback(std::bind(&RobotController::parameters_cb, this, std::placeholders::_1));
        }

    private:
        // ======================================== parameters ========================================
        OnSetParametersCallbackHandle::SharedPtr parameter_cb_;
        std::string aruco_marker_0_;
        std::string aruco_marker_1_;
        std::string aruco_marker_2_;

        // ======================================== attributes ========================================
        // Shared pointer to Utils Class.
        std::shared_ptr<Utils> utils_ptr_;

        // Publishers
        rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr turtle_camera_subscription_;
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_camera_subscription_;
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

        // Broadcasters
        std::shared_ptr<tf2_ros::TransformBroadcaster> aruco_tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr aruco_broadcast_timer_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr part_broadcast_timer_;

        // Listeners
        std::shared_ptr<tf2_ros::TransformListener> aruco_tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> aruco_tf_buffer_;
        rclcpp::TimerBase::SharedPtr aruco_listener_timer_;

        std::shared_ptr<tf2_ros::TransformListener> part_tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> part_tf_buffer_;
        rclcpp::TimerBase::SharedPtr part_listener_timer_;

        // Robot Attributes
        std::array<double, 3> robot_position_;
        geometry_msgs::msg::Quaternion robot_orientation_;

        // Marker Attributes
        std::string marker_id_;
        double dist_2_nearest_aruco_{100};

        // Navigation Logic Attributes
        std::string turn_instruction_;
        int turn_ctr_{0};
        bool in_turn_{false};
        bool at_finish_{false};

        // Part Info Storage
        std::string part_type_;
        std::string part_color_;

        // Current Time
        rclcpp::Time current_time_;

        // Storage for all Detected Parts
        std::vector<std::tuple<std::string, std::string, std::array<double, 3>, geometry_msgs::msg::Quaternion>> detected_parts_;

        // ======================================== methods ===========================================

        /**
         * @brief Timer callback to publish cmd_vel messages
         *
         */
        void cmd_vel_timer_cb();

        /**
         * @brief Subscriber callback to update current position of turtlebot.
         * @param msg
         */

        void odom_sub_cb_(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
         * @brief Method to calculate the distance between two objects (Robot and Detected Aruco Marker)
         * @param loc1, loc2
         */

        double calculate_distance(const std::array<double, 3> &loc1, const std::array<double, 3> &loc2);

        /**
         * @brief Convert a part type to a string
         *
         * @param part_type
         * @return battery
         * @return regulator
         * @return sensor
         * @return pump
         * @return unknown
         */
        std::string convert_part_type_to_string(uint8_t part_type);

        /**
         * @brief Convert a part color to a string
         *
         * @param part_color
         * @return red
         * @return green
         * @return blue
         * @return purple
         * @return orange
         * @return unknown
         */
        std::string convert_part_color_to_string(uint8_t part_color);

        /**
         * @brief Timer callback to broadcast aruco pose to tf.
         * @param msg
         *
         */
        void aruco_broadcast_timer_cb_(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        /**
         * @brief Subscriber callback to update aruco marker location.
         * @param msg
         */
        void turtle_camera_sub_cb_(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        /**
         * @brief Timer callback to broadcast part pose to tf.
         * @param msg
         *
         */
        void part_broadcast_timer_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Subscriber callback to update part location.
         * @param msg
         */
        void advanced_camera_sub_cb_(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        /**
         * @brief Subscriber callback to update current time.
         * @param msg
         */

        void clock_sub_cb_(const rosgraph_msgs::msg::Clock::SharedPtr msg);

        /**
         * @brief Method to listen for transformation updates for aruco markers.
         */
        void aruco_frame_listener_();

        /**
         * @brief Method to listen for transformation updates for parts.
         */
        void part_frame_listener_();

        /**
         * @brief Method to check parameters for directional turn instructions.
         */

        void check_turn_instruction();

        /**
         * @brief Method to ensure parts seen are unique.
         * @param new_part_color
         * @param new_part_type
         * @return true
         * @return false
         */
        bool check_duplicate_parts(const std::string &new_part_color, const std::string &new_part_type);

        /**
         * @brief Method to add seen parts to data structure.
         * @param color
         * @param type
         * @param position
         * @param orientation
         */
        void add_seen_part(const std::string &color, const std::string &type, const std::array<double, 3> &position, const geometry_msgs::msg::Quaternion &orientation);

        /**
         * @brief Method to print all seen parts.
         */
        void print_seen_parts();

        /**
         * @brief Parameter callback to watch for changes of parameters.
         * @param parameters
         */
        rcl_interfaces::msg::SetParametersResult parameters_cb(const std::vector<rclcpp::Parameter> &parameters);

    }; // Class Robot Controller
} // Namespace RWA3