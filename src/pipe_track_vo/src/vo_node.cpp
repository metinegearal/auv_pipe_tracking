#include "pipe_track_vo/vo_node.hpp"
#include <opencv2/imgproc.hpp>

namespace pipe_track_vo {

VoNode::VoNode()
: Node("vo_node"),
  current_depth_(1.0),
  first_frame_(true)
{
  this->declare_parameter<std::string>("camera_topic", "object/mask");
  this->declare_parameter<std::string>("depth_topic", "holocean/depth/distance");
  this->declare_parameter<std::string>("odom_topic", "perception/vo_odom");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->declare_parameter<double>("fx", 144.0);
  this->declare_parameter<double>("fy", 144.0);
  this->declare_parameter<double>("cx", 144.0);
  this->declare_parameter<double>("cy", 144.0);
  this->declare_parameter<int>("max_features", 300);
  this->declare_parameter<double>("default_scale", 0.05);
  this->declare_parameter<bool>("use_depth_scaling", true);

  this->get_parameter("camera_topic", camera_topic_);
  this->get_parameter("depth_topic", depth_topic_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("default_scale", default_scale_);
  this->get_parameter("use_depth_scaling", use_depth_scaling_);

  double fx = this->get_parameter("fx").as_double();
  double fy = this->get_parameter("fy").as_double();
  double cx = this->get_parameter("cx").as_double();
  double cy = this->get_parameter("cy").as_double();
  int max_feat = this->get_parameter("max_features").as_int();

  vo_engine_ = std::make_unique<VisualOdometry>(fx, fy, cx, cy, max_feat, 0.20);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_topic_, 10, std::bind(&VoNode::image_callback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    depth_topic_, 10, std::bind(&VoNode::depth_callback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  debug_img_pub_ = image_transport::create_publisher(this, "perception/vo_debug");

  RCLCPP_INFO(this->get_logger(), "Visual Odometry Node Initialized.");
  RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", camera_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing VO to: %s", odom_topic_.c_str());
}

void VoNode::depth_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  if (msg->data > 0.05f) {
    current_depth_ = static_cast<double>(msg->data);
  }
}

void VoNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  rclcpp::Time current_stamp = msg->header.stamp;

  if (first_frame_) {
    last_frame_time_ = current_stamp;
    first_frame_ = false;
    vo_engine_->process_frame(cv_ptr->image, 0.033, default_scale_);
    return;
  }

  double dt = (current_stamp - last_frame_time_).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    dt = 0.033;
  }
  last_frame_time_ = current_stamp;

  double effective_scale = default_scale_;
  if (use_depth_scaling_ && current_depth_ > 0.1) {
    effective_scale = default_scale_ * (current_depth_ / 2.0);
  }

  bool success = vo_engine_->process_frame(cv_ptr->image, dt, effective_scale);

  if (success && vo_engine_->is_initialized()) {
    publish_odometry(current_stamp);
  }

  publish_debug_image(cv_ptr->image, current_stamp);
}

void VoNode::publish_odometry(const rclcpp::Time & stamp) {
  const auto & state = vo_engine_->get_state();

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;

  msg.pose.pose.position.x = state.position.x();
  msg.pose.pose.position.y = state.position.y();
  msg.pose.pose.position.z = state.position.z();

  msg.pose.pose.orientation.w = state.orientation.w();
  msg.pose.pose.orientation.x = state.orientation.x();
  msg.pose.pose.orientation.y = state.orientation.y();
  msg.pose.pose.orientation.z = state.orientation.z();

  msg.pose.covariance[0] = 0.01;
  msg.pose.covariance[7] = 0.01;
  msg.pose.covariance[14] = 0.01;
  msg.pose.covariance[21] = 0.005;
  msg.pose.covariance[28] = 0.005;
  msg.pose.covariance[35] = 0.005;

  msg.twist.twist.linear.x = state.linear_velocity.x();
  msg.twist.twist.linear.y = state.linear_velocity.y();
  msg.twist.twist.linear.z = state.linear_velocity.z();

  msg.twist.twist.angular.x = state.angular_velocity.x();
  msg.twist.twist.angular.y = state.angular_velocity.y();
  msg.twist.twist.angular.z = state.angular_velocity.z();

  odom_pub_->publish(msg);
}

void VoNode::publish_debug_image(const cv::Mat & current_image, const rclcpp::Time & stamp) {
  if (debug_img_pub_.getNumSubscribers() == 0) {
    return;
  }

  cv::Mat debug_bgr;
  cv::cvtColor(current_image, debug_bgr, cv::COLOR_GRAY2BGR);

  const auto & prev_pts = vo_engine_->get_tracker().get_previous_points();
  const auto & curr_pts = vo_engine_->get_tracker().get_current_points();
  const auto & inliers = vo_engine_->get_tracker().get_inlier_mask();

  for (size_t i = 0; i < curr_pts.size() && i < prev_pts.size(); ++i) {
    bool is_inlier = (i < inliers.size()) && (inliers[i] != 0);
    cv::Scalar color = is_inlier ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

    cv::circle(debug_bgr, curr_pts[i], 3, color, -1);
    cv::line(debug_bgr, prev_pts[i], curr_pts[i], color, 1);
  }

  sensor_msgs::msg::Image::SharedPtr out_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_bgr).toImageMsg();
  out_msg->header.stamp = stamp;
  out_msg->header.frame_id = base_frame_id_;

  debug_img_pub_.publish(out_msg);
}

} // namespace pipe_track_vo


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pipe_track_vo::VoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}