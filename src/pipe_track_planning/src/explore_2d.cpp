#include "utils/explore_2d.hpp"

#include <iostream>

#include "utils/math_utils.hpp"

Explore2D::Explore2D() {}

double Explore2D::expSmooth(double point, std::optional<double> old_value)
{
  if (!old_value.has_value()) return point;
  return ALPHA_ * point + (1.0 - ALPHA_) * old_value.value();
}

std::pair<cv::Point2f, int> Explore2D::camera_to_world_coordinates(
  double px, double py, double depth, double fov, cv::Size img_size)
{
  auto local_pt = math_utils::point_from_depth({px, py}, yaw_, depth, true);
  int status = 1;

  if (
    px < img_size.width * (1.0 - SAFE_FRAME_) || px >= img_size.width * SAFE_FRAME_ ||
    py < img_size.height * (1.0 - SAFE_FRAME_) || py >= img_size.height * SAFE_FRAME_) {
    status = 2;
  }

  return {cv::Point2f(local_pt[0] + world_point_.x, local_pt[1] + world_point_.y), status};
}

void Explore2D::update_final_map()
{
  for (const auto & pt : point_map_scanned_) {
    int x = std::get<0>(pt), y = std::get<1>(pt);
    double status = std::get<2>(pt);
    auto key = std::make_pair(x, y);

    if (final_map_.find(key) == final_map_.end()) {
      final_map_[key] = status;
      if (status == 2.0) point_map_scanning_.insert(key);
    } else {
      double current_status = final_map_[key];
      if (status != 2.0) {
        if (current_status == 2.0) {
          final_map_[key] = status;
          point_map_scanning_.erase(key);
        } else {
          final_map_[key] = expSmooth(status, current_status);
        }
      }
    }
  }
  point_map_scanned_.clear();
}

void Explore2D::calculate_ideal_path()
{
  if (actual_path_.empty()) return;

  std::vector<cv::Point2f> pipe_cells;
  for (const auto & [key, value] : final_map_) {
    if (value >= 0.1 && value != 2.0) {
      pipe_cells.emplace_back(key.first, key.second);
    }
  }

  if (pipe_cells.empty()) {
    ideal_path_ = actual_path_;
    return;
  }

  double search_radius = 2.0 / MAP_RESOLUTION;
  std::vector<cv::Point2f> raw_ideal_path;

  for (const auto & wp : actual_path_) {
    cv::Point2f current_pos(wp.x / MAP_RESOLUTION, wp.y / MAP_RESOLUTION);
    std::vector<cv::Point2f> local_cells;

    for (const auto & cell : pipe_cells) {
      if (cv::norm(cell - current_pos) <= search_radius) {
        local_cells.push_back(cell);
      }
    }

    if (!local_cells.empty()) {
      cv::Point2f center(0, 0);
      for (const auto & c : local_cells) center += c;
      center.x = (center.x / local_cells.size()) * MAP_RESOLUTION;
      center.y = (center.y / local_cells.size()) * MAP_RESOLUTION;
      raw_ideal_path.push_back(center);
    } else {
      if (!raw_ideal_path.empty())
        raw_ideal_path.push_back(raw_ideal_path.back());
      else
        raw_ideal_path.push_back(wp);
    }
  }

  ideal_path_.clear();
  int window = 3;
  for (size_t i = 0; i < raw_ideal_path.size(); ++i) {
    int start = std::max(0, (int)i - window);
    int end = std::min((int)raw_ideal_path.size(), (int)i + window + 1);
    cv::Point2f sum(0, 0);
    for (int j = start; j < end; ++j) sum += raw_ideal_path[j];
    ideal_path_.push_back(cv::Point2f(sum.x / (end - start), sum.y / (end - start)));
  }
}

void Explore2D::add_points(
  const std::vector<MapPoint> & points, cv::Size img_size, double depth,
  const cv::Point2f & position, double yaw)
{
  yaw_ = yaw;
  world_point_ = position;

  std::cout << "Adding points with position=[" << position.x << ", " << position.y
            << "] yaw=" << yaw << " depth=" << depth << "\n";
  actual_path_.push_back(position);

  for (const auto & p : points) {
    auto [world_pt, status] = camera_to_world_coordinates(p.x, p.y, depth, 90.0, img_size);
    double obj = p.obj;
    if (status == 2) {
      if (obj == 1.0)
        obj = status;
      else
        continue;
    }
    point_map_scanned_.push_back(
      {(int)(world_pt.x / MAP_RESOLUTION), (int)(world_pt.y / MAP_RESOLUTION), obj});
  }

  update_final_map();
  calculate_ideal_path();
  save_final_map();
}

cv::Mat Explore2D::get_map()
{
  if (final_map_.empty() && actual_path_.empty()) return cv::Mat();

  int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
  for (const auto & [key, val] : final_map_) {
    min_x = std::min(min_x, key.first);
    max_x = std::max(max_x, key.first);
    min_y = std::min(min_y, key.second);
    max_y = std::max(max_y, key.second);
  }

  for (const auto & path : {actual_path_, ideal_path_}) {
    for (const auto & p : path) {
      int px = p.x / MAP_RESOLUTION, py = p.y / MAP_RESOLUTION;
      min_x = std::min(min_x, px);
      max_x = std::max(max_x, px);
      min_y = std::min(min_y, py);
      max_y = std::max(max_y, py);
    }
  }

  int width = max_x - min_x + 1;
  int height = max_y - min_y + 1;
  if (width <= 0 || height <= 0) return cv::Mat();

  cv::Mat img(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

  for (const auto & [key, status] : final_map_) {
    cv::Vec3b color(0, 0, 0);
    if (status == 2.0)
      color = cv::Vec3b(255, 255, 0);
    else if (status >= 0.1)
      color = cv::Vec3b(255, 0, 0);
    img.at<cv::Vec3b>(max_y - key.second, key.first - min_x) = color;
  }

  return img;
}

std::string Explore2D::save_final_map()
{
  cv::Mat img = get_map();
  if (img.empty()) return "";

  std::string output_path = "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/final_map.png";
  cv::imwrite(output_path, img);
  return output_path;
}