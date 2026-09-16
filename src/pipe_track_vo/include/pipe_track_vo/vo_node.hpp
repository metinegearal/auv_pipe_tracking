#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include "pipe_track_vo/visual_odometry.hpp"

namespace pipe_track_vo {

class VoNode : public rclcpp::Node {
public:
  VoNode();
  ~VoNode() = default;

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depth_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void publish_odometry(const rclcpp::Time & stamp);
  void publish_debug_image(const cv::Mat & current_image, const rclcpp::Time & stamp);

  std::unique_ptr<VisualOdometry> vo_engine_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  image_transport::Publisher debug_img_pub_;

  double current_depth_;
  rclcpp::Time last_frame_time_;
  bool first_frame_;

  std::string camera_topic_;
  std::string depth_topic_;
  std::string odom_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double default_scale_;
  bool use_depth_scaling_;
};

} // namespace pipe_track_vo