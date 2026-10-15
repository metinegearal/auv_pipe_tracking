#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>  // <-- Add this for BT signals
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "utils/explore_2d.hpp"
#include "utils/map_target.hpp"

class PointExtractNavi : public rclcpp::Node
{
public:
  PointExtractNavi() : Node("point_extract_navi_node")
  {
    sub_cam_ = this->create_subscription<sensor_msgs::msg::Image>(
      "object/mask", 10, std::bind(&PointExtractNavi::cam_callback, this, std::placeholders::_1));
    position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "estimation/odom", 10,
      std::bind(&PointExtractNavi::position_callback, this, std::placeholders::_1));
    height_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/distance", 10,
      std::bind(&PointExtractNavi::depth_callback, this, std::placeholders::_1));
    sub_mag_ = this->create_subscription<std_msgs::msg::Float32>(
      "perception/yaw", 10,
      std::bind(&PointExtractNavi::yaw_callback, this, std::placeholders::_1));

    waypoint_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("/trajectory/waypoint", 10);

    // --- NEW: Publisher for the Behavior Tree ---
    exploration_empty_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/planning/exploration_empty", 10);

    RCLCPP_INFO(this->get_logger(), "✅ point_extract_navi Started (Pure Map-Based)");
  }

private:
  void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ = cv::Point2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
  }

  void depth_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_height_ = msg->data; }
  void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    yaw_ = msg->data;
    is_not_started_ = false;
  }

  void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (is_stopped_ || is_not_started_ || (current_position_.x == 0 && current_position_.y == 0))
      return;

    cv::Mat mask = cv_bridge::toCvCopy(msg, "mono8")->image;
    int h = mask.rows;
    int w = mask.cols;

    std::vector<MapPoint> points;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (mask.at<uint8_t>(y, x) > 0)
          points.push_back({x, y, 1.0});
        else
          points.push_back({x, y, 0.0});
      }
    }

    // --- MAP INTEGRATION ---
    explorer_.add_points(points, cv::Size(w, h), current_height_, current_position_, yaw_);

    // Frontier points are cell indices; the selector returns a target in metres.
    const auto frontier_cells = explorer_.get_scan_point();
    const cv::Point2f target_point = target_selector_.select(
      frontier_cells, current_position_, explorer_.MAP_RESOLUTION);
    bool is_exploration_empty = false;  // Default for BT

    if (!frontier_cells.empty()) {
      // We successfully extracted points. Exploration is officially running.
      has_started_exploration_ = true;
    } else {
      if (has_started_exploration_) {
        is_exploration_empty = true;
        RCLCPP_WARN(this->get_logger(), "🏁 Map fully explored! Sending stop signal to BT.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Waiting for initial exploration points...");
      }
    }

    // Publish to Behavior Tree
    std_msgs::msg::Bool bt_msg;
    bt_msg.data = is_exploration_empty;
    exploration_empty_pub_->publish(bt_msg);

    RCLCPP_INFO(this->get_logger(), "Map Target: (%f, %f)", target_point.x, target_point.y);

    geometry_msgs::msg::PointStamped wp_msg;
    wp_msg.header.stamp = msg->header.stamp;
    wp_msg.header.frame_id = "map";
    wp_msg.point.x = target_point.x;
    wp_msg.point.y = target_point.y;
    wp_msg.point.z = -28.0;
    waypoint_pub_->publish(wp_msg);

    // --- MAP VISUALIZATION ---
    cv::Mat map_img = explorer_.get_map();
    if (!map_img.empty()) {
      cv::Mat resized_map;
      cv::resize(map_img, resized_map, cv::Size(500, 500), 0, 0, cv::INTER_NEAREST);
      cv::imshow("2D Map Explorer", resized_map);
    }

    cv::Mat debug_mask;
    cv::cvtColor(mask, debug_mask, cv::COLOR_GRAY2BGR);
    cv::imshow("Segmentation Mask", debug_mask);

    cv::waitKey(1);
  }

  Explore2D explorer_;
  cv::Point2f current_position_{0, 0};
  double yaw_ = 0;
  double current_height_ = 0;
  bool is_stopped_ = false;
  bool is_not_started_ = true;

  bool has_started_exploration_ = false;
  map_navigation::TargetSelector target_selector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_, sub_mag_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_empty_pub_;  // BT Publisher
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointExtractNavi>());
  rclcpp::shutdown();
  return 0;
}
