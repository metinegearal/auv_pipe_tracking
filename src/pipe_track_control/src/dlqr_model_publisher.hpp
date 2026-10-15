#pragma once

// Publish the exact design used by the controller for out-of-process analysis.
#include "dlqr_utils.hpp"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace pipe_track_dlqr
{
template<typename Derived>
nlohmann::json matrix_json(const Eigen::MatrixBase<Derived> & matrix)
{
  auto rows = nlohmann::json::array();
  for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
    auto row = nlohmann::json::array();
    for (Eigen::Index j = 0; j < matrix.cols(); ++j) {row.push_back(matrix(i, j));}
    rows.push_back(row);
  }
  return rows;
}

inline rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publish_model(
  rclcpp::Node & node, const FeedbackSystem & system, const Settings & settings,
  const std::string & mode, double cruise_speed)
{
  auto publisher = node.create_publisher<std_msgs::msg::String>(
    "/motion/dlqr_model", rclcpp::QoS(1).reliable().transient_local());
  const nlohmann::json model = {
    {"schema_version", 1}, {"dt", settings.dt}, {"mass", settings.mass},
    {"damping", settings.damping}, {"perfect", settings.perfect},
    {"control_mode", mode}, {"waypoint_cruise_speed", cruise_speed},
    {"state_order", {"vx", "vy", "vz", "px", "py", "pz"}},
    {"A", matrix_json(system.a())}, {"B", matrix_json(system.b())},
    {"K_state", matrix_json(system.state_gain())},
    {"K_velocity", matrix_json(system.velocity_gain())}};
  std_msgs::msg::String message;
  message.data = model.dump();
  publisher->publish(message);
  return publisher;
}
}  // namespace pipe_track_dlqr
