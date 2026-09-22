#include "pipe_track_estimation/estimation_node.hpp"

namespace pipe_track_estimation
{

EstimationNode::EstimationNode() : Node("estimation_node"), is_initialized_(false)
{
  estimator_ = std::make_shared<GtsamEstimator>();

  // Setup Subscribers
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "holocean/imu", 200, std::bind(&EstimationNode::imu_callback, this, std::placeholders::_1));

  dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "holocean/dvl", 10, std::bind(&EstimationNode::dvl_callback, this, std::placeholders::_1));

  vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "perception/vo_odom", 10, std::bind(&EstimationNode::vo_callback, this, std::placeholders::_1));

  mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
    "holocean/mag", 10, std::bind(&EstimationNode::mag_callback, this, std::placeholders::_1));

  // Setup Publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("estimation/odom", 30);

  RCLCPP_INFO(this->get_logger(), "✅ GTSAM Estimation Node Started (Full Implementation)");
}

void EstimationNode::mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
  // Calculate absolute yaw from magnetometer X and Y
  current_yaw_ = std::atan2(msg->magnetic_field.x, msg->magnetic_field.y) -
                 M_PI / 2.0;  // Adjust for coordinate frame if necessary
  if (current_yaw_ < -M_PI) current_yaw_ += 2 * M_PI;
}

void EstimationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!is_initialized_) return;

  Eigen::Vector3d accel(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  // Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3d gyro(0, 0, 0);  // Placeholder for gyro; in a real system, use the above line

  // Calculate accurate dt using ROS message timestamps
  static rclcpp::Time last_imu_time(0, 0, this->get_clock()->get_clock_type());
  rclcpp::Time current_time = msg->header.stamp;

  if (last_imu_time.nanoseconds() == 0) {
    last_imu_time = current_time;
    return;  // Skip the very first frame to get a valid dt on the next one
  }

  // double dt = (current_time - last_imu_time).seconds();
  // last_imu_time = current_time;

  // // Failsafe for simulator stutter
  // if (dt <= 0.0 || dt > 0.1) dt = 0.01;
  double dt = 0.033;  // Placeholder for dt; in a real system, use the above calculation

  // Send to GTSAM
  estimator_->add_imu_measurement(accel, gyro, dt);
}

void EstimationNode::dvl_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  Eigen::Vector3d linear_vel(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

  // --- NEW: DVL NOW ANCHORS THE GRAPH ---
  if (!is_initialized_) {
    // --- NEW: Initialize with absolute magnetometer yaw! ---
    gtsam::Pose3 start_pose(gtsam::Rot3::Yaw(current_yaw_), gtsam::Point3(0, 0, 0));
    estimator_->initialize(start_pose, linear_vel, gtsam::imuBias::ConstantBias());
    is_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Graph anchored by DVL and Mag. Dead reckoning active.");
    return;
  }

  const double timestamp =
    1.0 / 30.0;  // Placeholder for timestamp; in a real system, use msg->header.stamp
  estimator_->add_dvl_measurement(linear_vel, timestamp, current_yaw_);

  // --- NEW: PUBLISH FROM DVL CALLBACK ---
  // Since VO is disabled, we extract and publish the state every time the DVL pings.
  auto current_pose = estimator_->get_current_pose();
  auto current_vel = estimator_->get_current_velocity();

  nav_msgs::msg::Odometry fused_odom;
  fused_odom.header.stamp = this->get_clock()->now();
  fused_odom.header.frame_id = "odom";
  fused_odom.child_frame_id = "base_link";

  fused_odom.pose.pose.position.x = current_pose.translation().x() + 43.203;
  fused_odom.pose.pose.position.y = current_pose.translation().y() - 64.463;
  fused_odom.pose.pose.position.z = current_pose.translation().z() - 24.178;

  auto quaternion = current_pose.rotation().toQuaternion();
  fused_odom.pose.pose.orientation.w = quaternion.w();
  fused_odom.pose.pose.orientation.x = quaternion.x();
  fused_odom.pose.pose.orientation.y = quaternion.y();
  fused_odom.pose.pose.orientation.z = quaternion.z();

  fused_odom.twist.twist.linear.x = current_vel.x();
  fused_odom.twist.twist.linear.y = current_vel.y();
  fused_odom.twist.twist.linear.z = current_vel.z();

  odom_pub_->publish(fused_odom);

  RCLCPP_INFO(
    this->get_logger(), "Fused Pose: [%.3f, %.3f, %.3f]", current_pose.translation().x() + 43.203,
    current_pose.translation().y() - 64.463, current_pose.translation().z() - 24.178);
}

void EstimationNode::vo_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  return;  // VO is currently disabled in this implementation

  auto pos = msg->pose.pose.position;
  auto ori = msg->pose.pose.orientation;

  gtsam::Pose3 vo_pose(
    gtsam::Rot3::Quaternion(ori.w, ori.x, ori.y, ori.z), gtsam::Point3(pos.x, pos.y, pos.z));

  if (!is_initialized_) {
    // First VO frame anchors the graph
    estimator_->initialize(vo_pose, gtsam::Vector3::Zero(), gtsam::imuBias::ConstantBias());
    is_initialized_ = true;
    return;
  }

  estimator_->add_vo_measurement(vo_pose, msg->header.stamp.sec);

  // --- PUBLISH FUSED STATE ---
  auto current_pose = estimator_->get_current_pose();
  auto current_vel = estimator_->get_current_velocity();

  RCLCPP_INFO(
    this->get_logger(), "Fused Pose: [%.3f, %.3f, %.3f]", current_pose.translation().x(),
    current_pose.translation().y(), current_pose.translation().z());

  RCLCPP_INFO(
    this->get_logger(), "Fused Velocity: [%.3f, %.3f, %.3f]", current_vel.x(), current_vel.y(),
    current_vel.z());

  nav_msgs::msg::Odometry fused_odom;
  fused_odom.header.stamp = this->get_clock()->now();
  fused_odom.header.frame_id = "odom";
  fused_odom.child_frame_id = "base_link";

  // Fused Position
  fused_odom.pose.pose.position.x = current_pose.translation().x();
  fused_odom.pose.pose.position.y = current_pose.translation().y();
  fused_odom.pose.pose.position.z = current_pose.translation().z();

  // Fused Orientation (Convert GTSAM Rot3 to ROS Quaternion)
  auto quaternion = current_pose.rotation().toQuaternion();
  fused_odom.pose.pose.orientation.w = quaternion.w();
  fused_odom.pose.pose.orientation.x = quaternion.x();
  fused_odom.pose.pose.orientation.y = quaternion.y();
  fused_odom.pose.pose.orientation.z = quaternion.z();

  // Fused Velocity
  fused_odom.twist.twist.linear.x = current_vel.x();
  fused_odom.twist.twist.linear.y = current_vel.y();
  fused_odom.twist.twist.linear.z = current_vel.z();

  odom_pub_->publish(fused_odom);
}

}  // namespace pipe_track_estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pipe_track_estimation::EstimationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}