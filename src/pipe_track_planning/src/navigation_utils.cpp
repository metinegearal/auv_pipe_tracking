#include "utils/navigation_utils.hpp"

#include <cmath>

namespace navigation_utils
{

// Helper function for Zhang-Suen thinning
void thinningIteration(cv::Mat & img, int iter)
{
  cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);
  for (int i = 1; i < img.rows - 1; i++) {
    for (int j = 1; j < img.cols - 1; j++) {
      uchar p2 = img.at<uchar>(i - 1, j);
      uchar p3 = img.at<uchar>(i - 1, j + 1);
      uchar p4 = img.at<uchar>(i, j + 1);
      uchar p5 = img.at<uchar>(i + 1, j + 1);
      uchar p6 = img.at<uchar>(i + 1, j);
      uchar p7 = img.at<uchar>(i + 1, j - 1);
      uchar p8 = img.at<uchar>(i, j - 1);
      uchar p9 = img.at<uchar>(i - 1, j - 1);

      int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + (p4 == 0 && p5 == 1) +
              (p5 == 0 && p6 == 1) + (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
              (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
      int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
      int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
      int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

      if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0) marker.at<uchar>(i, j) = 1;
    }
  }
  img &= ~marker;
}

// Topologically preserving skeletonization (Matches skimage)
void skeletonize(const cv::Mat & src, cv::Mat & dst)
{
  dst = src.clone();
  dst /= 255;  // Convert 255 to 1 for the logic gates

  cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
  cv::Mat diff;

  do {
    thinningIteration(dst, 0);
    thinningIteration(dst, 1);
    cv::absdiff(dst, prev, diff);
    dst.copyTo(prev);
  } while (cv::countNonZero(diff) > 0);

  dst *= 255;  // Convert back to 255 for display/masks
}

std::tuple<double, cv::Point, cv::Mat, cv::Mat> get_weighted_pipe_navigation(
  const cv::Mat & mask_image, int look_ahead_pixels, double angle_weight, bool use_skeleton)
{
  int height = mask_image.rows;
  int width = mask_image.cols;
  int center_x = width / 2 - 20;
  int center_y = height / 2 + 30;

  cv::Mat binary_mask;
  cv::threshold(mask_image, binary_mask, 127, 255, cv::THRESH_BINARY);

  cv::Mat clean_skeleton = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
  std::vector<cv::Point> points;

  if (use_skeleton) {
    cv::Mat skeleton;
    skeletonize(binary_mask, skeleton);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(skeleton, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if (!contours.empty()) {
      auto largest_contour = std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> & a, const std::vector<cv::Point> & b) {
          return cv::arcLength(a, false) < cv::arcLength(b, false);
        });
      cv::drawContours(
        clean_skeleton, std::vector<std::vector<cv::Point>>{*largest_contour}, -1, cv::Scalar(255),
        1);
      cv::findNonZero(clean_skeleton, points);
    }
  } else {
    binary_mask.copyTo(clean_skeleton);
    cv::findNonZero(clean_skeleton, points);
  }

  if (points.empty()) {
    return {
      0.0, cv::Point(center_x, center_y), clean_skeleton,
      cv::Mat::zeros(clean_skeleton.size(), CV_64F)};
  }

  double best_cost = std::numeric_limits<double>::max();
  cv::Point best_point = points[0];
  cv::Mat weight_map = cv::Mat::zeros(clean_skeleton.size(), CV_64F);
  std::vector<double> costs(points.size());

  double max_cost = -1.0;
  double min_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < points.size(); ++i) {
    double dx = points[i].x - center_x;
    double dy = center_y - points[i].y;

    double dist = std::hypot(dx, dy);
    double angle = std::abs(std::atan2(dx, dy));

    double cost_dist = std::abs(dist - look_ahead_pixels);
    double cost_angle = angle * look_ahead_pixels * angle_weight;
    double total_cost = cost_dist + cost_angle;

    costs[i] = total_cost;
    if (total_cost < best_cost) {
      best_cost = total_cost;
      best_point = points[i];
    }
    if (total_cost > max_cost) max_cost = total_cost;
    if (total_cost < min_cost) min_cost = total_cost;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    double w = (max_cost - min_cost > 1e-6) ? (max_cost - costs[i]) / (max_cost - min_cost) : 1.0;
    weight_map.at<double>(points[i].y, points[i].x) = w;
  }

  double tdx = best_point.x - center_x;
  double tdy = center_y - best_point.y;
  double final_angle_deg = std::atan2(tdx, tdy) * 180.0 / M_PI;

  return {final_angle_deg, best_point, clean_skeleton, weight_map};
}

MultiNavResult get_multi_pipe_navigation(
  const cv::Mat & mask_image, const std::vector<int> & look_aheads, double angle_weight,
  bool use_skeleton)
{
  int center_x = mask_image.cols / 2 - 20;
  int center_y = mask_image.rows / 2 + 30;

  // 1. Heavy CV operations happen ONLY ONCE
  cv::Mat binary_mask, clean_skeleton;
  cv::threshold(mask_image, binary_mask, 127, 255, cv::THRESH_BINARY);
  clean_skeleton = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
  std::vector<cv::Point> points;

  if (use_skeleton) {
    cv::Mat skeleton;
    skeletonize(binary_mask, skeleton);  // Assuming your custom skeletonize is in scope

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(skeleton, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if (!contours.empty()) {
      auto largest_contour = std::max_element(
        contours.begin(), contours.end(),
        [](const std::vector<cv::Point> & a, const std::vector<cv::Point> & b) {
          return cv::arcLength(a, false) < cv::arcLength(b, false);
        });
      cv::drawContours(
        clean_skeleton, std::vector<std::vector<cv::Point>>{*largest_contour}, -1, cv::Scalar(255),
        1);
      cv::findNonZero(clean_skeleton, points);
    }
  } else {
    binary_mask.copyTo(clean_skeleton);
    cv::findNonZero(clean_skeleton, points);
  }

  // 2. Initialize tracking variables for ALL look-aheads
  size_t num_targets = look_aheads.size();
  std::vector<cv::Point> best_points(num_targets, cv::Point(center_x, center_y));
  std::vector<double> best_costs(num_targets, std::numeric_limits<double>::max());
  std::vector<double> final_angles(num_targets, 0.0);

  if (points.empty()) {
    return {final_angles, best_points, clean_skeleton};
  }

  // 3. Single iteration over pixels
  for (const auto & pt : points) {
    double dx = pt.x - center_x;
    double dy = center_y - pt.y;  // Image Y is inverted

    // Cache the math so we don't recalculate it for every lookahead
    double dist = std::hypot(dx, dy);
    double angle = std::abs(std::atan2(dx, dy));

    // Evaluate this pixel against all requested look-ahead distances
    for (size_t i = 0; i < num_targets; ++i) {
      double cost_dist = std::abs(dist - look_aheads[i]);
      double cost_angle = angle * look_aheads[i] * angle_weight;
      double total_cost = cost_dist + cost_angle;

      if (total_cost < best_costs[i]) {
        best_costs[i] = total_cost;
        best_points[i] = pt;
      }
    }
  }

  // 4. Compute final angles based on the winning points
  for (size_t i = 0; i < num_targets; ++i) {
    double tdx = best_points[i].x - center_x;
    double tdy = center_y - best_points[i].y;
    final_angles[i] = std::atan2(tdx, tdy) * 180.0 / M_PI;
  }

  return {final_angles, best_points, clean_skeleton};
}

}  // namespace navigation_utils