#include "pipe_track_vo/feature_tracker.hpp"
#include <opencv2/imgproc.hpp>

namespace pipe_track_vo {

FeatureTracker::FeatureTracker(int max_features, double quality_level, double min_distance)
: max_features_(max_features),
  quality_level_(quality_level),
  min_distance_(min_distance),
  is_initialized_(false)
{
}

void FeatureTracker::detect_features(const cv::Mat & image, std::vector<cv::Point2f> & points) {
  cv::goodFeaturesToTrack(
    image,
    points,
    max_features_,
    quality_level_,
    min_distance_,
    cv::noArray(),
    3,
    false,
    0.04
  );
}

bool FeatureTracker::track(const cv::Mat & current_image,
                           const cv::Mat & camera_matrix,
                           cv::Mat & relative_rotation,
                           cv::Mat & relative_translation)
{
  if (!is_initialized_) {
    detect_features(current_image, prev_points_);
    if (prev_points_.size() < 30) {
      return false;
    }
    prev_image_ = current_image.clone();
    is_initialized_ = true;
    return false;
  }

  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(
    prev_image_,
    current_image,
    prev_points_,
    curr_points_,
    status,
    err,
    cv::Size(21, 21),
    3,
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
    0,
    1e-4
  );

  std::vector<cv::Point2f> matched_prev;
  std::vector<cv::Point2f> matched_curr;
  matched_prev.reserve(curr_points_.size());
  matched_curr.reserve(curr_points_.size());

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      if (curr_points_[i].x >= 0 && curr_points_[i].x < current_image.cols &&
          curr_points_[i].y >= 0 && curr_points_[i].y < current_image.rows)
      {
        matched_prev.push_back(prev_points_[i]);
        matched_curr.push_back(curr_points_[i]);
      }
    }
  }

  if (matched_curr.size() < 15) {
    detect_features(current_image, prev_points_);
    prev_image_ = current_image.clone();
    return false;
  }

  cv::Mat inlier_mask_mat;
  cv::Mat E = cv::findEssentialMat(
    matched_curr,
    matched_prev,
    camera_matrix,
    cv::RANSAC,
    0.999,
    1.0,
    inlier_mask_mat
  );

  if (E.empty() || E.rows != 3 || E.cols != 3) {
    detect_features(current_image, prev_points_);
    prev_image_ = current_image.clone();
    return false;
  }

  cv::Mat R, t;
  int inliers = cv::recoverPose(
    E,
    matched_curr,
    matched_prev,
    camera_matrix,
    R,
    t,
    inlier_mask_mat
  );

  if (inliers < 10) {
    detect_features(current_image, prev_points_);
    prev_image_ = current_image.clone();
    return false;
  }

  inlier_mask_.clear();
  inlier_mask_.assign(inlier_mask_mat.begin<uchar>(), inlier_mask_mat.end<uchar>());

  relative_rotation = R.clone();
  relative_translation = t.clone();

  if (matched_curr.size() < static_cast<size_t>(max_features_ * 0.5)) {
    std::vector<cv::Point2f> new_features;
    detect_features(current_image, new_features);
    for (const auto & pt : new_features) {
      matched_curr.push_back(pt);
    }
  }

  prev_points_ = matched_curr;
  prev_image_ = current_image.clone();

  return true;
}

} // namespace pipe_track_vo