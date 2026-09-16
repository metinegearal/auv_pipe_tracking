#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "pipe_track_estimation/gtsam_estimator.hpp"

namespace pipe_track_estimation {

class EstimationNode : public rclcpp::Node {
public:
    EstimationNode();

private:
    // Callbacks
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void vo_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Math Engine
    std::shared_ptr<GtsamEstimator> estimator_;

    // Initialization flag
    bool is_initialized_;

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

} // namespace pipe_track_estimation