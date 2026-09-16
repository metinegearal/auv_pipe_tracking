#include <algorithm>
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "math_pid_utils.hpp"

using std::placeholders::_1;

class WaypointControlNode : public rclcpp::Node
{
public:
  WaypointControlNode()
  : Node("waypoint_control_node"),
    pid_dist_(0.75, 0.3, 0.1, 0.0, 0.2),
    pid_yaw_(0.6, 0.3, 0.1, 0.0, 0.2)
  {
    // --- Declare and Get Parameters from config/params.yaml ---
    this->declare_parameter<double>("kp_yaw", 0.6);
    this->declare_parameter<double>("kp_dist", 0.75);
    this->declare_parameter<bool>("dvl_active", false);

    this->get_parameter("kp_yaw", kp_yaw_);
    this->get_parameter("kp_dist", kp_dist_);
    this->get_parameter("dvl_active", dvl_active_);

    waypoint_ = {0.0, 0.0, -26.0};
    magnet_ = {0.0, 0.0, 0.0};
    obstacles_.resize(9, 0.0);
    motor_command_.resize(8, 0.0);
    last_base_command_.resize(8, 0.0);
    last_motor_command_.resize(8, 0.0);

    // Publishers
    target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/holo/cmd", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motion/report", 10);

    // Subscribers
    waypoint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/trajectory/waypoint", 10, std::bind(&WaypointControlNode::waypoint_callback, this, _1));
    position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "holocean/odom", 10, std::bind(&WaypointControlNode::position_callback, this, _1));
    distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/distance", 10, std::bind(&WaypointControlNode::dist_callback, this, _1));
    depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/scalar", 10, std::bind(&WaypointControlNode::depth_callback, this, _1));
    sub_mag_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "holocean/mag", 10, std::bind(&WaypointControlNode::magnetometer_callback, this, _1));
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "object/obstacle", 10, std::bind(&WaypointControlNode::obstacle_callback, this, _1));
    angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/aruco/angle", 10, std::bind(&WaypointControlNode::angle_callback, this, _1));
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/movement/mode", 10, std::bind(&WaypointControlNode::mode_callback, this, _1));
    finished_execution_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/movement/finished_execution", 10,
      std::bind(&WaypointControlNode::finished_execution_callback, this, _1));
    dvl_reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/movement/dvl_reset", 10, std::bind(&WaypointControlNode::dvl_reset_callback, this, _1));

    move_towards_waypoint({10.0, 10.0, 5.0});
    RCLCPP_INFO(this->get_logger(), "✅ Waypoint Control Node Started (Cleaned Edition)");
  }

private:
  std::vector<double> waypoint_;
  std::vector<double> magnet_;
  std::vector<float> obstacles_;
  std::vector<double> motor_command_;

  double current_yaw_ = 0.0;
  double original_yaw_ = 0.0;
  double current_depth_ = 0.0;
  double current_dist_seabed_ = 0.0;

  utils::PID pid_dist_;
  utils::PID pid_yaw_;

  double kp_dist_;
  double kp_yaw_;

  bool finished_execution_ = false;
  std::optional<double> hardcoded_angle_ = std::nullopt;
  std::optional<double> fixed_angle_ = std::nullopt;
  std::string mode_ = "yaw_centerize";

  std::vector<double> last_base_command_;
  std::vector<double> last_motor_command_;

  std::vector<double> current_position_ = {0.0, 0.0, 0.0};
  std::vector<double> dvl_position_ = {0.0, 0.0, 0.0};
  std::optional<std::vector<double>> dvl_reset_position_ = std::nullopt;
  bool dvl_active_;  // Assuming True based on Python fallback

  // --- Publishers ---
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr stop_pub_;

  // --- Subscribers ---
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_sub_, obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_, depth_sub_, angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_execution_sub_, dvl_reset_sub_;

  double get_time_now() { return this->get_clock()->now().seconds(); }

  void publish_cmd(std::vector<double> motor_command)
  {
    last_base_command_ = motor_command;
    last_motor_command_ = motor_command;

    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(motor_command.begin(), motor_command.end());
    target_pub_->publish(msg);
  }

  // --- Original Callbacks ---
  void finished_execution_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    finished_execution_ = msg->data;
  }
  void mode_callback(const std_msgs::msg::String::SharedPtr msg) { mode_ = msg->data; }
  void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (mode_ == "no_centerize") hardcoded_angle_ = msg->data;
  }

  void dvl_reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      dvl_reset_position_ = dvl_position_;
      dvl_active_ = true;
      RCLCPP_INFO(this->get_logger(), "DVL position reset");
    }
  }

  void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto pos = msg->pose.pose.position;
    dvl_position_ = {pos.x, pos.y, 0.0};
    if (dvl_active_)
      current_position_ = {pos.x, pos.y, pos.z};
    else
      current_position_ = {0.0, 0.0, pos.z};
    move_towards_waypoint(waypoint_);
  }

  void magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    magnet_ = {msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z};
    double ya = std::atan2(magnet_[0], magnet_[1]) - (M_PI / 2.0);
    if (ya < -M_PI) ya += M_PI * 2.0;
    current_yaw_ = ya;
    original_yaw_ = std::atan2(magnet_[0], magnet_[1]);
  }

  void depth_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_depth_ = msg->data; }
  void dist_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    current_dist_seabed_ = msg->data;
  }

  void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    obstacles_.assign(msg->data.begin(), msg->data.end());
  }

  void waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 3) {
      waypoint_ = {msg->data[0], msg->data[1], msg->data[2]};
      if (msg->data.size() >= 4) fixed_angle_ = msg->data[3];
      move_towards_waypoint(waypoint_);
    }
  }

  void move_towards_waypoint(std::vector<double> target)
  {
    if (finished_execution_) return;

    if (dvl_active_ && dvl_reset_position_.has_value()) {
      for (size_t i = 0; i < 3; ++i) target[i] += dvl_reset_position_.value()[i];
    }
    if (target[2] < 0) current_position_[2] = current_depth_;

    std::vector<double> direction(3);
    for (size_t i = 0; i < 3; ++i) direction[i] = target[i] - current_position_[i];
    double distance = std::sqrt(
      direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);

    // Stopping condition
    if (utils::diff_2d(target, current_position_) < 1.0) {
      std_msgs::msg::Float32MultiArray stop_msg;
      std::vector<float> f_target(target.begin(), target.end());
      stop_msg.data = f_target;
      stop_pub_->publish(stop_msg);
    }

    if (distance == 0.0) return;
    for (size_t i = 0; i < 3; ++i) direction[i] /= distance;

    double desired_yaw =
      fixed_angle_.has_value() ? fixed_angle_.value() : std::atan2(direction[1], direction[0]);
    double raw_yaw_diff = desired_yaw - current_yaw_;
    double yaw_error = hardcoded_angle_.has_value()
                         ? hardcoded_angle_.value()
                         : std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));

    // Fetch dynamic parameters
    this->get_parameter("kp_yaw", kp_yaw_);
    this->get_parameter("kp_dist", kp_dist_);

    std::vector<double> cmd(8, 0.0);
    double vert_mult = (mode_ == "full_centerize") ? 0.25 : 0.6;
    for (int i = 0; i < 4; i++) cmd[i] = vert_mult * motor_command_[i];

    // ==========================================
    // NEW: BLENDED CONTINUOUS CONTROL LOGIC
    // ==========================================

    // 1. Calculate Continuous Yaw (No Deadband)
    double yaw_command = 0.0;
    if (mode_ == "yaw_centerize" || mode_ == "full_centerize") {
      // YOUR original math: directly multiply error by gain
      yaw_command = kp_yaw_ * yaw_error;

      // Clamp it to maximum thrust
      yaw_command = std::clamp(yaw_command, -10.0, 10.0);

      // YOUR original inversion to match thruster polarity
      yaw_command = -yaw_command;
    }

    // 2. Calculate Dynamic Surge Speed (Cosine Scaling)
    // Map distance to a base thrust (the "carrot")
    double base_thrust = std::clamp(distance * kp_dist_, 1.0, 10.0);

    // Scale thrust based on how straight we are aiming to prevent corner overshoot
    // cos(yaw_error) smoothly drops the speed as the angle increases.
    // std::max(0.1, ...) ensures we never completely stop moving forward.
    double speed_scaling = std::max(0.1, std::cos(yaw_error));
    double forward_thrust = base_thrust * speed_scaling;

    // 3. Apply Positional Movement (Surge/Sway)
    auto auv_pos = utils::world_to_auv_coordinates(direction, 3.0 * M_PI / 2.0 - original_yaw_);
    auto keys = determine_keys(auv_pos);
    cmd = parse_keys(keys, forward_thrust, cmd);

    // 4. Overlay Yaw Command simultaneously
    if (mode_ == "yaw_centerize" || mode_ == "full_centerize") {
      // Note: Check if you need to invert yaw_command here based on your thruster layout
      cmd[4] -= yaw_command;
      cmd[7] -= yaw_command;
      cmd[5] += yaw_command;
      cmd[6] += yaw_command;
    }

    // ==========================================

    motor_command_ = cmd;
    std::vector<double> scaled_cmd = cmd;
    // for (auto & val : scaled_cmd) val *= 5.0; // Global multiplier

    publish_cmd(scaled_cmd);
  }

  std::vector<char> determine_keys(const std::vector<double> & dir)
  {
    std::vector<char> keys;
    if (obstacles_.size() > 4 && obstacles_[4] == 0.0) {
      if (dir[0] > 0.1)
        keys.push_back('w');
      else if (dir[0] < -0.1)
        keys.push_back('s');
      if (dir[1] > 0.1)
        keys.push_back('d');
      else if (dir[1] < -0.1)
        keys.push_back('a');
    } else {
      if (dir[0] > 0.1)
        keys.push_back('d');
      else if (dir[0] < -0.1)
        keys.push_back('a');
    }

    if (dir[2] > 0.1)
      keys.push_back('i');
    else if (dir[2] < -0.1)
      keys.push_back('k');
    return keys;
  }

  std::vector<double> parse_keys(
    const std::vector<char> & keys, double val, std::vector<double> cmd)
  {
    auto has_key = [&](char k) { return std::find(keys.begin(), keys.end(), k) != keys.end(); };

    if (has_key('i')) {
      for (int i = 0; i < 4; i++) cmd[i] += val;
    }
    if (has_key('k')) {
      for (int i = 0; i < 4; i++) cmd[i] -= val;
    }
    if (has_key('j')) {
      cmd[4] += val;
      cmd[7] += val;
      cmd[5] -= val;
      cmd[6] -= val;
    }
    if (has_key('l')) {
      cmd[4] -= val;
      cmd[7] -= val;
      cmd[5] += val;
      cmd[6] += val;
    }
    if (has_key('w')) {
      for (int i = 4; i < 8; i++) cmd[i] -= val;
    }
    if (has_key('s')) {
      for (int i = 4; i < 8; i++) cmd[i] += val;
    }
    if (has_key('a')) {
      cmd[4] += val;
      cmd[6] += val;
      cmd[5] -= val;
      cmd[7] -= val;
    }
    if (has_key('d')) {
      cmd[4] -= val;
      cmd[6] -= val;
      cmd[5] += val;
      cmd[7] += val;
    }
    return cmd;
  }

  std::vector<double> apply_yaw_control(double yaw_error, double val, std::vector<double> cmd)
  {
    double yaw_command = std::clamp(kp_yaw_ * yaw_error, -val, val);
    yaw_command = -yaw_command;
    cmd[4] -= yaw_command;
    cmd[7] -= yaw_command;
    cmd[5] += yaw_command;
    cmd[6] += yaw_command;
    return cmd;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointControlNode>());
  rclcpp::shutdown();
  return 0;
}