#pragma once
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

namespace navigation_utils
{

// Custom skeletonization since skimage is Python-only
void skeletonize(const cv::Mat & img, cv::Mat & skeleton);

std::tuple<double, cv::Point, cv::Mat, cv::Mat> get_weighted_pipe_navigation(
  const cv::Mat & mask_image, int look_ahead_pixels = 100, double angle_weight = 1.5,
  bool use_skeleton = true);

  struct MultiNavResult {
    std::vector<double> angles_deg;
    std::vector<cv::Point> centers;
    cv::Mat skeleton_img;
  };

  MultiNavResult get_multi_pipe_navigation(
    const cv::Mat & mask_image, 
    const std::vector<int>& look_aheads, 
    double angle_weight, 
  bool use_skeleton);

}  // namespace navigation_utils