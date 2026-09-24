#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

namespace pipe_track_vo
{

class FeatureTracker
{
public:
  FeatureTracker(int max_features = 400, double quality_level = 0.01, double min_distance = 10.0);
  ~FeatureTracker() = default;

  bool track(
    const cv::Mat & current_image, const cv::Mat & camera_matrix, cv::Mat & relative_rotation,
    cv::Mat & relative_translation);

  const std::vector<cv::Point2f> & get_previous_points() const { return prev_points_; }
  const std::vector<cv::Point2f> & get_current_points() const { return curr_points_; }
  const std::vector<uchar> & get_inlier_mask() const { return inlier_mask_; }

private:
  void detect_features(const cv::Mat & image, std::vector<cv::Point2f> & points);

  int max_features_;
  double quality_level_;
  double min_distance_;

  cv::Mat prev_image_;
  std::vector<cv::Point2f> prev_points_;
  std::vector<cv::Point2f> curr_points_;
  std::vector<uchar> inlier_mask_;
  bool is_initialized_;
};

}  // namespace pipe_track_vo