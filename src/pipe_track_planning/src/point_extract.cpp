#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <Eigen/Dense>

#include "utils/lie_algebra.hpp"
#include "utils/explore_2d.hpp"
#include "utils/navigation_utils.hpp"
#include "utils/math_utils.hpp"

class ObjectSegmentation : public rclcpp::Node {
public:
    ObjectSegmentation() : Node("object_segmentation_node") {
        
        // --- PROPER PARAMETER DECLARATION IN CONSTRUCTOR ---
        this->declare_parameter<int>("look_ahead_pixels", 100);
        this->declare_parameter<double>("angle_weight", 0.5);

        this->get_parameter("look_ahead_pixels", look_ahead_pixels_);
        this->get_parameter("angle_weight", angle_weight_);
        
        sub_cam_ = this->create_subscription<sensor_msgs::msg::Image>(
            "object/mask", 10, std::bind(&ObjectSegmentation::cam_callback, this, std::placeholders::_1));
        position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "holocean/odom", 10, std::bind(&ObjectSegmentation::position_callback, this, std::placeholders::_1));
        height_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "holocean/depth/distance", 10, std::bind(&ObjectSegmentation::depth_callback, this, std::placeholders::_1));
        sub_mag_ = this->create_subscription<std_msgs::msg::Float32>(
            "perception/yaw", 10, std::bind(&ObjectSegmentation::yaw_callback, this, std::placeholders::_1));

        center_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object/center", 10);
        waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/trajectory/waypoint", 10);

        RCLCPP_INFO(this->get_logger(), "✅ segmentation_node Started");
    }

private:
    void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_ = cv::Point2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
    
    void depth_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_height_ = msg->data; }
    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) { yaw_ = msg->data; }

    void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (is_stopped_) return;

        // Fetch dynamic parameters in case they were updated
        this->get_parameter("look_ahead_pixels", look_ahead_pixels_);
        this->get_parameter("angle_weight", angle_weight_);

        cv::Mat mask = cv_bridge::toCvCopy(msg, "mono8")->image;
        int h = mask.rows;
        int w = mask.cols;

        std::vector<MapPoint> points;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (mask.at<uint8_t>(y, x) > 0) points.push_back({x, y, 1.0});
                else points.push_back({x, y, 0.0});
            }
        }

        auto [angle, center, skel_img, weight_map] = 
            navigation_utils::get_weighted_pipe_navigation(mask, look_ahead_pixels_, angle_weight_, true);

        // FIXED: Explicit std::vector for Float32MultiArray
        std_msgs::msg::Float32MultiArray center_msg;
        std::vector<float> center_data = {static_cast<float>(center.x), static_cast<float>(center.y)};
        center_msg.data = center_data;
        center_pub_->publish(center_msg);

        // 1. Define your camera intrinsics (fx, fy, cx, cy)
        Eigen::Matrix3d K;
        K << 144.0,   0.0, 144.0,  
              0.0, 144.0, 144.0,
              0.0,   0.0,   1.0;

        Eigen::Matrix4d T_body_cam = Eigen::Matrix4d::Identity();
        T_body_cam.block<3, 3>(0, 0) <<  0.0, -1.0,  0.0,
                                        -1.0,  0.0,  0.0,
                                         0.0,  0.0, -1.0;

        // 2. Construct the SE(3) pose of the AUV body
        Eigen::Matrix<double, 6, 1> auv_state_xi;
        auv_state_xi << 0.0, 0.0, yaw_, 0.0, 0.0, current_height_; 
        Eigen::Matrix4d T_world_body = auv_geometry::exp_se3(auv_state_xi);

        // 3. Chain the transforms: World -> Body -> Camera
        Eigen::Matrix4d T_world_cam = T_world_body * T_body_cam;

        // 4. Get your world point seamlessly
        Eigen::Vector2d pixel(center.x, center.y);
        Eigen::Vector3d world_point = math_utils::point_from_depth_matrix(pixel, 15.0, K, T_world_cam);
        RCLCPP_INFO(this->get_logger(), "Center in image: (%d, %d), 3D point vectorized: (%f, %f)", center.x, center.y, world_point.x(), world_point.y());
        
        // FIXED: Explicit std::vector declaration for the old function
        std::vector<double> center_vec = {static_cast<double>(center.x), static_cast<double>(center.y)};
        auto point_3d = math_utils::point_from_depth(center_vec, yaw_, 15.0, true);
        RCLCPP_INFO(this->get_logger(), "Center in image: (%d, %d), 3D point: (%f, %f)", center.x, center.y, point_3d[0], point_3d[1]);

        explorer_.add_points(points, cv::Size(w, h), current_height_, current_position_, yaw_);

        // FIXED: Explicit std::vector for Float32MultiArray
        std_msgs::msg::Float32MultiArray wp_msg;
        std::vector<float> wp_data = {
            static_cast<float>(world_point.x()), 
            static_cast<float>(world_point.y()), 
            -28.0f
        };
        wp_msg.data = wp_data;
        waypoint_pub_->publish(wp_msg);

        // --- Debug Visualization ---
        cv::Mat debug_img;
        cv::cvtColor(skel_img, debug_img, cv::COLOR_GRAY2BGR);
        cv::Point img_center(w / 2, h / 2);
        cv::circle(debug_img, img_center, 5, cv::Scalar(255, 0, 0), -1);
        cv::circle(debug_img, center, 5, cv::Scalar(0, 255, 0), -1);
        cv::line(debug_img, img_center, center, cv::Scalar(0, 255, 255), 2);
        
        std::string filename = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/results/debug_" 
                             + std::to_string(msg->header.stamp.sec) + "_" 
                             + std::to_string(msg->header.stamp.nanosec) + ".jpg";
        cv::imwrite(filename, debug_img);
        
        cv::imshow("Segmentation Debug", debug_img);
        cv::waitKey(1);
    }

    Explore2D explorer_;
    cv::Point2f current_position_{0, 0};
    double yaw_ = 0;
    double current_height_ = 0;
    bool is_stopped_ = false;

    // Parameter variables
    int look_ahead_pixels_ = 100;
    double angle_weight_ = 0.5;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_, sub_mag_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr center_pub_, waypoint_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSegmentation>());
    rclcpp::shutdown();
    return 0;
}