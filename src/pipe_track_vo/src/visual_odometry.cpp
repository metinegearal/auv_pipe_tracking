#include "pipe_track_vo/visual_odometry.hpp"

#include <iostream>

namespace pipe_track_vo
{

VisualOdometry::VisualOdometry(
  double fx, double fy, double cx, double cy, int max_features, double min_inlier_ratio)
: min_inlier_ratio_(min_inlier_ratio), initialized_(false)
{
  K_ = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

  tracker_ = std::make_unique<FeatureTracker>(max_features, 0.01, 10.0);
  reset();
}

void VisualOdometry::reset()
{
  current_rotation_matrix_ = Eigen::Matrix3d::Identity();
  current_translation_vector_ = Eigen::Vector3d::Zero();

  state_.position = Eigen::Vector3d::Zero();
  state_.orientation = Eigen::Quaterniond::Identity();
  state_.linear_velocity = Eigen::Vector3d::Zero();
  state_.angular_velocity = Eigen::Vector3d::Zero();

  initialized_ = false;
}

bool VisualOdometry::process_frame(const cv::Mat & image, double dt, double scale_estimate)
{
  cv::Mat R_rel_cv, t_rel_cv;
  bool tracked = tracker_->track(image, K_, R_rel_cv, t_rel_cv);

  if (!tracked) {
    state_.linear_velocity = Eigen::Vector3d::Zero();
    state_.angular_velocity = Eigen::Vector3d::Zero();
    return false;
  }

  Eigen::Matrix3d R_rel;
  R_rel << R_rel_cv.at<double>(0, 0), R_rel_cv.at<double>(0, 1), R_rel_cv.at<double>(0, 2),
    R_rel_cv.at<double>(1, 0), R_rel_cv.at<double>(1, 1), R_rel_cv.at<double>(1, 2),
    R_rel_cv.at<double>(2, 0), R_rel_cv.at<double>(2, 1), R_rel_cv.at<double>(2, 2);

  Eigen::Vector3d t_rel;
  t_rel << t_rel_cv.at<double>(0), t_rel_cv.at<double>(1), t_rel_cv.at<double>(2);

  double step_scale = (scale_estimate > 0.001) ? scale_estimate : 1.0;
  Eigen::Vector3d delta_t_world = current_rotation_matrix_ * (t_rel * step_scale);

  current_translation_vector_ += delta_t_world;
  current_rotation_matrix_ = current_rotation_matrix_ * R_rel.transpose();

  state_.position = current_translation_vector_;
  state_.orientation = Eigen::Quaterniond(current_rotation_matrix_);
  state_.orientation.normalize();

  if (dt > 1e-5) {
    state_.linear_velocity = delta_t_world / dt;
    Eigen::AngleAxisd angle_axis(R_rel.transpose());
    state_.angular_velocity = angle_axis.axis() * angle_axis.angle() / dt;
  } else {
    state_.linear_velocity = Eigen::Vector3d::Zero();
    state_.angular_velocity = Eigen::Vector3d::Zero();
  }

  initialized_ = true;
  return true;
}

}  // namespace pipe_track_vo