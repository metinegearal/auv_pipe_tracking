#pragma once
#include <vector>
#include <map>
#include <set>
#include <string>
#include <opencv2/opencv.hpp>
#include <optional>

struct MapPoint {
    int x, y;
    double obj;
};

class Explore2D {
public:
    Explore2D();
    
    cv::Mat get_map();
    // std::vector<cv::Point2f> get_scan_point() const { return ideal_path_; }
    std::vector<cv::Point2f> get_scan_point() const { 
        std::vector<cv::Point2f> pts;
        for(auto& p : point_map_scanning_) pts.push_back(cv::Point2f(p.first, p.second));
        return pts;
    }

    void add_points(const std::vector<MapPoint>& points, cv::Size img_size, double depth, const cv::Point2f& position, double yaw);
    std::string save_final_map();
    
    double MAP_RESOLUTION = 0.25;

private:
    double expSmooth(double point, std::optional<double> old_value);
    std::pair<cv::Point2f, int> camera_to_world_coordinates(double px, double py, double depth, double fov, cv::Size img_size);
    void update_final_map();
    void calculate_ideal_path();
    void add_point(const MapPoint& point, cv::Size img_size, double depth);
    
    std::vector<std::tuple<int, int, double>> point_map_scanned_;
    std::set<std::pair<int, int>> point_map_scanning_;
    std::map<std::pair<int, int>, double> final_map_;

    double yaw_ = 0.0;
    cv::Point2f world_point_{0, 0};
    double SAFE_FRAME_ = 0.9;
    double ALPHA_ = 0.1;

    std::vector<cv::Point2f> actual_path_;
    std::vector<cv::Point2f> ideal_path_;
};