#pragma once

#include <Eigen/Dense>
#include <memory>
#include <opencv2/core.hpp>

#include "pipe_track_vo/feature_tracker.hpp"

namespace pipe_track_vo
{

struct OdometryState
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;
};

class VisualOdometry
{
public:
  VisualOdometry(
    double fx, double fy, double cx, double cy, int max_features = 400,
    double min_inlier_ratio = 0.20);
  ~VisualOdometry() = default;

  bool process_frame(const cv::Mat & image, double dt, double scale_estimate);

  const OdometryState & get_state() const { return state_; }
  const FeatureTracker & get_tracker() const { return *tracker_; }
  bool is_initialized() const { return initialized_; }
  void reset();

private:
  cv::Mat K_;
  std::unique_ptr<FeatureTracker> tracker_;
  OdometryState state_;

  Eigen::Matrix3d current_rotation_matrix_;
  Eigen::Vector3d current_translation_vector_;

  double min_inlier_ratio_;
  bool initialized_;
};

}  // namespace pipe_track_vo