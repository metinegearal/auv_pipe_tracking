#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
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
        height_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "holocean/depth/distance", 10, std::bind(&ObjectSegmentation::depth_callback, this, std::placeholders::_1));
        sub_mag_ = this->create_subscription<std_msgs::msg::Float32>(
            "perception/yaw", 10, std::bind(&ObjectSegmentation::yaw_callback, this, std::placeholders::_1));

        center_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object/center", 10);
        waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/trajectory/waypoint", 10);
        
        // NEW: Turn back publisher for the Behavior Tree
        turn_back_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planning/turn_back", 10);

        RCLCPP_INFO(this->get_logger(), "✅ segmentation_node Started");
    }

private:
    void depth_callback(const std_msgs::msg::Float32::SharedPtr msg) { current_height_ = msg->data; }
    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg) { yaw_ = msg->data; }

    void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (is_stopped_) return;

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

        std_msgs::msg::Float32MultiArray center_msg;
        std::vector<float> center_data = {static_cast<float>(center.x), static_cast<float>(center.y)};
        center_msg.data = center_data;
        center_pub_->publish(center_msg);

        // 1. Define your camera intrinsics
        Eigen::Matrix3d K;
        K << 144.0,   0.0, 144.0,  
              0.0, 144.0, 144.0,
              0.0,   0.0,   1.0;

        Eigen::Matrix4d T_body_cam = Eigen::Matrix4d::Identity();
        T_body_cam.block<3, 3>(0, 0) <<  0.0, -1.0,  0.0,
                                        -1.0,  0.0,  0.0,
                                         0.0,  0.0, -1.0;

        // 2. Construct the SE(3) pose
        Eigen::Matrix<double, 6, 1> auv_state_xi;
        auv_state_xi << 0.0, 0.0, yaw_, 0.0, 0.0, current_height_; 
        Eigen::Matrix4d T_world_body = auv_geometry::exp_se3(auv_state_xi);

        // 3. Chain transforms and get world point
        Eigen::Matrix4d T_world_cam = T_world_body * T_body_cam;
        Eigen::Vector2d pixel(center.x, center.y);
        Eigen::Vector3d world_point = math_utils::point_from_depth_matrix(pixel, 15.0, K, T_world_cam);

        // --- NEW: BEHAVIOR TREE TURN BACK LOGIC ---
        // Calculate angle between our current yaw and the target waypoint
        double desired_yaw = std::atan2(world_point.y(), world_point.x());
        double raw_diff = desired_yaw - yaw_;
        
        // Wrap difference to [-180, 180] degrees
        double yaw_diff_deg = std::abs(std::atan2(std::sin(raw_diff), std::cos(raw_diff))) * 180.0 / M_PI;

        // Condition 3: Ignore start of tracking. Wait until AUV is aligned (< 20 degrees) for at least 20 frames.
        if (!tracking_stabilized_) {
            if (yaw_diff_deg < 20.0) {
                stabilization_counter_++;
                if (stabilization_counter_ > 20) {
                    tracking_stabilized_ = true;
                    RCLCPP_INFO(this->get_logger(), "🚀 Tracking stabilized! Turn-back safety active.");
                }
            } else {
                stabilization_counter_ = 0; // Reset if it wanders off
            }
        }

        // Conditions 1 & 2: If we are stabilized, check for >60 degree turns
        bool trigger_turn_back = false;
        
        if (tracking_stabilized_) {
            if (yaw_diff_deg > 60.0) {
                extreme_turn_counter_++;
            } else {
                extreme_turn_counter_ = 0; // Gate it out: reset if it was just a glitch
            }

            // Must see the extreme turn for 10 consecutive frames (~1 second at 10Hz)
            if (extreme_turn_counter_ > 10) {
                trigger_turn_back = true;
                RCLCPP_WARN(this->get_logger(), "⚠️ Turn Back triggered! Yaw diff: %.1f deg", yaw_diff_deg);
            }
        }

        // Publish to Behavior Tree
        std_msgs::msg::Bool tb_msg;
        tb_msg.data = trigger_turn_back;
        turn_back_pub_->publish(tb_msg);
        // ------------------------------------------

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
        
        cv::imshow("Segmentation Debug", debug_img);
        cv::waitKey(1);
    }

    Explore2D explorer_;
    double yaw_ = 0;
    double current_height_ = 0;
    bool is_stopped_ = false;
    
    // BT Logic Variables
    bool tracking_stabilized_ = false;
    int stabilization_counter_ = 0;
    int extreme_turn_counter_ = 0;

    int look_ahead_pixels_ = 100;
    double angle_weight_ = 0.5;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_, sub_mag_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr center_pub_, waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr turn_back_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSegmentation>());
    rclcpp::shutdown();
    return 0;
}