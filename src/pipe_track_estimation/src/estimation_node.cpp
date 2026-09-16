#include "pipe_track_estimation/estimation_node.hpp"

namespace pipe_track_estimation {

EstimationNode::EstimationNode() : Node("estimation_node"), is_initialized_(false) {
    estimator_ = std::make_shared<GtsamEstimator>();

    // Setup Subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "holocean/imu", 200, std::bind(&EstimationNode::imu_callback, this, std::placeholders::_1));
    
    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "holocean/dvl", 10, std::bind(&EstimationNode::dvl_callback, this, std::placeholders::_1));

    vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "perception/vo_odom", 10, std::bind(&EstimationNode::vo_callback, this, std::placeholders::_1));

    // Setup Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("estimation/odom", 30);
    
    RCLCPP_INFO(this->get_logger(), "✅ GTSAM Estimation Node Started (Full Implementation)");
}

void EstimationNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!is_initialized_) return;

    Eigen::Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    
    // Hardcoded dt for ~200Hz. (In a real system, calculate this using msg->header.stamp)
    estimator_->add_imu_measurement(accel, gyro, 0.005); 
}

void EstimationNode::dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    if (!is_initialized_) return;

    // Extract velocity from the DVL message
    Eigen::Vector3d linear_vel(msg->twist.twist.linear.x, 
                               msg->twist.twist.linear.y, 
                               msg->twist.twist.linear.z);

    estimator_->add_dvl_measurement(linear_vel, msg->header.stamp.sec);
}

void EstimationNode::vo_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto pos = msg->pose.pose.position;
    auto ori = msg->pose.pose.orientation;
    
    gtsam::Pose3 vo_pose(gtsam::Rot3::Quaternion(ori.w, ori.x, ori.y, ori.z), 
                         gtsam::Point3(pos.x, pos.y, pos.z));

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

    RCLCPP_INFO(this->get_logger(), "Fused Pose: [%.3f, %.3f, %.3f]", 
                current_pose.translation().x(), 
                current_pose.translation().y(), 
                current_pose.translation().z());

    RCLCPP_INFO(this->get_logger(), "Fused Velocity: [%.3f, %.3f, %.3f]", 
                current_vel.x(), 
                current_vel.y(), 
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

} // namespace pipe_track_estimation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pipe_track_estimation::EstimationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}