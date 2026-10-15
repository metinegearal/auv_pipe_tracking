// Alternative to waypoint_control_node; run only one publisher of /holo/cmd.
#include <array>
#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "dlqr_model_publisher.hpp"
#include "dlqr_utils.hpp"

using pipe_track_dlqr::State;
using pipe_track_dlqr::Thrusters;

class DlqrControlNode : public rclcpp::Node
{
public:
  DlqrControlNode() : Node("dlqr_control_node")
  {
    pipe_track_dlqr::Settings settings;
    settings.dt = fixed_parameter("update_period", settings.dt);
    settings.mass = fixed_parameter("mass", settings.mass);
    settings.damping = fixed_parameter("damping", settings.damping);
    settings.perfect = fixed_parameter("perfect", settings.perfect);
    settings.state_weights = weights<6>("state_weights", {80., 80., 80., 16., 16., 16.});
    settings.velocity_weights = weights<3>("velocity_weights", {10., 10., 10.});
    settings.force_weights = weights<3>("force_weights", {0.1, 0.1, 0.1});
    sensor_timeout_ = fixed_parameter("sensor_timeout", 0.5);
    waypoint_timeout_ = fixed_parameter("waypoint_timeout", 1.0);
    coordinates_ = fixed_parameter<std::string>("waypoint_coordinates", "relative_xy");
    control_mode_ = fixed_parameter<std::string>("control_mode", "state");
    use_depth_ = fixed_parameter("use_depth_sensor", true);
    depth_sign_ = fixed_parameter("depth_sign", 1.0);
    cruise_speed_ = fixed_parameter("waypoint_cruise_speed", 0.0);
    braking_acceleration_ = fixed_parameter("waypoint_braking_acceleration", 0.8);
    stopping_distance_ = fixed_parameter("stopping_distance", 0.25);
    kp_yaw_ = fixed_parameter("kp_yaw", 0.6);
    max_yaw_torque_ = fixed_parameter("max_yaw_torque", 10.0);
    max_motor_force_ = fixed_parameter("max_motor_force", 40.0);
    const auto odom_topic = fixed_parameter<std::string>("odom_topic", "estimation/odom");
    validate(settings.dt);
    system_ = std::make_unique<pipe_track_dlqr::FeedbackSystem>(settings);
    allocator_ = std::make_unique<pipe_track_dlqr::ThrusterAllocator>(settings.perfect);
    model_pub_ =
      pipe_track_dlqr::publish_model(*this, *system_, settings, control_mode_, cruise_speed_);

    command_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/holo/cmd", 10);
    report_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/motion/report", 10);
    force_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>("/motion/commanded_force", 10);
    applied_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>("/motion/applied_force", 10);
    const auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        const Eigen::Vector3d position(p.x, p.y, p.z);
        if (!position.allFinite()) {
          received_[Odom].reset();
          return;
        }
        position_ = position;
        received_[Odom] = now().seconds();
        if (pending_origin_reset_) {
          reset_origin();
        }
      });
    // DVL is body velocity. The estimator currently publishes world velocity in
    // its twist despite child_frame_id=base_link; do not rotate that twist twice.
    dvl_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "holocean/dvl", qos, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const auto & v = msg->twist.linear;
        const Eigen::Vector3d velocity(v.x, v.y, v.z);
        if (!velocity.allFinite()) {
          received_[Dvl].reset();
          return;
        }
        body_velocity_ = velocity;
        received_[Dvl] = now().seconds();
      });
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      "holocean/mag", qos, [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
        const auto & m = msg->magnetic_field;
        if (!std::isfinite(m.x) || !std::isfinite(m.y) || std::hypot(m.x, m.y) < 1e-9) {
          received_[Mag].reset();
          return;
        }
        // Equivalent to pipe tracking's atan2(mx,my)-pi/2, wrapped to [-pi,pi].
        yaw_ = std::atan2(-m.y, m.x);
        rotation_ = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        received_[Mag] = now().seconds();
      });
    depth_sub_ = create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/scalar", qos, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          received_[Depth].reset();
          return;
        }
        depth_ = depth_sign_ * msg->data;
        received_[Depth] = now().seconds();
      });
    waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/trajectory/waypoint", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) { waypoint_callback(*msg); });
    finished_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/movement/finished_execution", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        finished_ = msg->data;
        if (finished_) {
          clear_target();
        }
      });
    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/movement/dvl_reset", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (!msg->data) {
          return;
        }
        if (fresh(received_[Odom], now().seconds(), sensor_timeout_)) {
          reset_origin();
        } else {
          pending_origin_reset_ = true;
        }
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/movement/mode", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        movement_mode_ = msg->data;
        angle_override_.reset();
      });
    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/aruco/angle", 10, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        if (movement_mode_ == "no_centerize" && std::isfinite(msg->data)) {
          angle_override_ = msg->data;  // Radians, matching the existing controller.
        }
      });
    obstacle_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "object/obstacle", 10, [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        obstacle_ahead_ =
          msg->data.size() > 4 && (!std::isfinite(msg->data[4]) || msg->data[4] != 0.f);
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(settings.dt), [this]() { tick(); });
    RCLCPP_INFO(
      get_logger(), "DLQR ready: %s waypoints, %s feedback, waiting for sensors/target",
      coordinates_.c_str(), control_mode_.c_str());
  }

  void stop() { publish_command(Thrusters::Zero()); }

private:
  enum Sensor
  {
    Odom,
    Dvl,
    Mag,
    Depth
  };

  template <typename T>
  T fixed_parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  template <int N>
  Eigen::Matrix<double, N, 1> weights(
    const std::string & name, const std::vector<double> & defaults)
  {
    const auto values = fixed_parameter(name, defaults);
    if (values.size() != N) {
      throw std::invalid_argument(name + " has incorrect size");
    }
    Eigen::Matrix<double, N, 1> result;
    for (int i = 0; i < N; ++i) {
      result[i] = values[i];
    }
    return result;
  }

  void validate(double dt) const
  {
    if (
      !std::isfinite(sensor_timeout_) || sensor_timeout_ <= dt ||
      !std::isfinite(waypoint_timeout_) || waypoint_timeout_ < 0. ||
      !std::isfinite(cruise_speed_) || cruise_speed_ < 0. ||
      !std::isfinite(braking_acceleration_) || braking_acceleration_ <= 0. ||
      !std::isfinite(stopping_distance_) || stopping_distance_ < 0. || !std::isfinite(kp_yaw_) ||
      kp_yaw_ < 0. || !std::isfinite(max_yaw_torque_) || max_yaw_torque_ < 0. ||
      !std::isfinite(max_motor_force_) || max_motor_force_ <= 0. ||
      max_motor_force_ > pipe_track_dlqr::ThrusterAllocator::max_thrust ||
      (depth_sign_ != 1. && depth_sign_ != -1.) ||
      (coordinates_ != "relative_xy" && coordinates_ != "world" && coordinates_ != "dvl_origin") ||
      (control_mode_ != "state" && control_mode_ != "velocity") ||
      (control_mode_ == "velocity" && cruise_speed_ <= 0.)) {
      throw std::invalid_argument("Invalid DLQR node parameters");
    }
  }

  static bool fresh(const std::optional<double> & stamp, double current, double timeout)
  {
    return stamp && current >= *stamp && (timeout == 0. || current - *stamp <= timeout);
  }

  void reset_origin()
  {
    origin_ = position_;
    origin_->z() = 0.;  // Targets always specify absolute z, as in the PID node.
    pending_origin_reset_ = false;
    if (coordinates_ == "dvl_origin") {
      clear_target();
    }
  }

  void clear_target()
  {
    target_.reset();
    waypoint_received_.reset();
    report_sent_ = false;
    stop();
  }

  void waypoint_callback(const geometry_msgs::msg::PointStamped & msg)
  {
    if (finished_) {
      return;
    }
    Eigen::Vector3d target(msg.point.x, msg.point.y, msg.point.z);
    const double current = now().seconds();
    if (
      !target.allFinite() ||
      (coordinates_ != "world" && !fresh(received_[Odom], current, sensor_timeout_)) ||
      (coordinates_ == "dvl_origin" && !origin_)) {
      clear_target();
      return;
    }
    // point_extract's header names the camera, but its numeric x/y are already
    // rotated into world axes. This explicit parameter describes the numeric
    // contract; no TF transform is applied to the misleading camera header.
    if (coordinates_ == "relative_xy") {
      target.head<2>() += position_.head<2>();
    }
    if (coordinates_ == "dvl_origin") {
      target += *origin_;
    }
    if (!target_ || (target - *target_).norm() > 1e-3) {
      report_sent_ = false;
    }
    target_ = target;
    waypoint_received_ = current;
  }

  void tick()
  {
    const double current = now().seconds();
    if (last_tick_ && current < *last_tick_) {
      for (auto & stamp : received_) {
        stamp.reset();
      }
      origin_.reset();
      clear_target();
    }
    last_tick_ = current;
    bool ready = target_ && fresh(waypoint_received_, current, waypoint_timeout_);
    for (int sensor = Odom; sensor <= Mag; ++sensor) {
      ready = ready && fresh(received_[sensor], current, sensor_timeout_);
    }
    if (use_depth_) {
      ready = ready && fresh(received_[Depth], current, sensor_timeout_);
    }
    if (finished_ || !ready) {
      stop();
      return;
    }

    // *** main calculation *** //
    State state;
    state.head<3>() = rotation_ * body_velocity_;
    state.tail<3>() = position_;
    if (use_depth_) {
      state[5] = depth_;
    }
    Eigen::Vector3d goal = *target_;
    const Eigen::Vector3d delta = goal - state.tail<3>();
    const double yaw_error = angle_override_.value_or(std::atan2(
      std::sin(std::atan2(delta.y(), delta.x()) - yaw_),
      std::cos(std::atan2(delta.y(), delta.x()) - yaw_)));
    const bool steer = movement_mode_ == "yaw_centerize" || movement_mode_ == "full_centerize" ||
                       (movement_mode_ == "no_centerize" && angle_override_);
    const double torque = steer && delta.head<2>().norm() > 1e-6
                            ? std::clamp(kp_yaw_ * yaw_error, -max_yaw_torque_, max_yaw_torque_)
                            : 0.;
    // Stop horizontal translation at an obstacle while damping actual velocity.
    if (obstacle_ahead_) {
      goal.head<2>() = state.segment<2>(3);
    }
    Eigen::Vector3d force;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    if (control_mode_ == "velocity" || cruise_speed_ > 0.) {
      velocity = pipe_track_dlqr::waypoint_velocity(
        state.tail<3>(), goal, cruise_speed_, braking_acceleration_, stopping_distance_);
      if (steer) {
        velocity.head<2>() *= std::max(0.1, std::cos(yaw_error));
      }
    }
    if (control_mode_ == "velocity" || velocity.norm() > 1e-9) {
      force = system_->velocity_force(state, velocity);
    } else {
      State reference = State::Zero();
      reference.tail<3>() = goal;
      force = system_->state_force(state, reference);
    }
    State wrench = State::Zero();
    wrench.head<3>() = rotation_.transpose() * force;
    wrench[5] = torque;
    Thrusters command = allocator_->allocate(wrench);

    // Uniform scaling preserves the requested wrench direction at saturation.
    const double peak = command.cwiseAbs().maxCoeff();
    if (peak > max_motor_force_) {
      command *= max_motor_force_ / peak;
    }
    if (!command.allFinite()) {
      stop();
      return;
    }
    publish_command(command);
    publish_force(force_pub_, force);
    const State applied_wrench = allocator_->mapping() * command;
    publish_force(applied_pub_, rotation_ * applied_wrench.head<3>());
    if (!report_sent_ && delta.norm() <= stopping_distance_) {
      std_msgs::msg::Float32MultiArray report;
      for (int i = 0; i < 3; ++i) {
        report.data.push_back(static_cast<float>((*target_)[i]));
      }
      report_pub_->publish(report);
      report_sent_ = true;
    }
  }

  void publish_command(const Thrusters & command)
  {
    std_msgs::msg::Float32MultiArray msg;
    for (int i = 0; i < 8; ++i) {
      msg.data.push_back(static_cast<float>(command[i]));
    }
    command_pub_->publish(msg);
  }

  void publish_force(
    const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr & publisher,
    const Eigen::Vector3d & force)
  {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.vector.x = force.x();
    msg.vector.y = force.y();
    msg.vector.z = force.z();
    publisher->publish(msg);
  }

  std::unique_ptr<pipe_track_dlqr::FeedbackSystem> system_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_pub_;
  std::unique_ptr<pipe_track_dlqr::ThrusterAllocator> allocator_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  std::optional<Eigen::Vector3d> target_, origin_;
  std::array<std::optional<double>, 4> received_;
  std::optional<double> waypoint_received_, last_tick_, angle_override_;
  double yaw_ = 0., depth_ = 0.;
  double sensor_timeout_, waypoint_timeout_, depth_sign_, cruise_speed_, braking_acceleration_;
  double stopping_distance_, kp_yaw_, max_yaw_torque_, max_motor_force_;
  bool use_depth_, finished_ = false, report_sent_ = false;
  bool obstacle_ahead_ = false, pending_origin_reset_ = false;
  std::string coordinates_, control_mode_, movement_mode_ = "yaw_centerize";
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_, report_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr force_pub_, applied_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_, angle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_, reset_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DlqrControlNode>();
    rclcpp::spin(node);
    if (rclcpp::ok()) {
      node->stop();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dlqr_control_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
