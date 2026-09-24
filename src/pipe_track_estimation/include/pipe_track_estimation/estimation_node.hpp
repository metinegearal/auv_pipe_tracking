#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "pipe_track_estimation/gtsam_estimator.hpp"

namespace pipe_track_estimation
{

class EstimationNode : public rclcpp::Node
{
public:
  EstimationNode();

private:
  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void dvl_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void vo_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  // Store the latest yaw
  double current_yaw_ = 0.0;
  rclcpp::Time last_imu_time_;
  bool first_imu_ = true;

  // Math Engine
  std::shared_ptr<GtsamEstimator> estimator_;

  // Initialization flag
  bool is_initialized_;

  // ROS 2 Interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
};

}  // namespace pipe_track_estimation