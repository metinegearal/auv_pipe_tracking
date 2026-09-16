#pragma once

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

namespace trajectory_opt
{

// --- Existing function from earlier ---
inline cv::Point optimize_trajectory(
  const cv::Point & origin, const std::vector<cv::Point> & lookahead_pts, double t = 0.2)
{
  if (lookahead_pts.size() < 3) return lookahead_pts.empty() ? origin : lookahead_pts[0];

  cv::Point2f p0(origin.x, origin.y);
  cv::Point2f p1(lookahead_pts[0].x, lookahead_pts[0].y);
  cv::Point2f p2(lookahead_pts[1].x, lookahead_pts[1].y);
  cv::Point2f p3(lookahead_pts[2].x, lookahead_pts[2].y);

  double u = 1.0 - t;
  double tt = t * t;
  double uu = u * u;
  double uuu = uu * u;
  double ttt = tt * t;

  cv::Point2f smoothed_pt = (uuu * p0) + (3 * uu * t * p1) + (3 * u * tt * p2) + (ttt * p3);
  return cv::Point(std::round(smoothed_pt.x), std::round(smoothed_pt.y));
}

// --- NEW: Dynamic Lookahead Calculator ---
/**
 * @brief Adjusts Bezier 't' parameter based on pipe curvature.
 * Straight pipe = higher t (looks further ahead, maintains high momentum).
 * Curved pipe = lower t (looks closer, prevents corner-cutting).
 */
inline double calculate_dynamic_lookahead(
  double pipe_angle_deg, double min_t = 0.70, double max_t = 0.95)
{
  // Cap the maximum expected angle to 90 degrees for scaling purposes
  double abs_angle = std::min(std::abs(pipe_angle_deg), 90.0);

  // Linear interpolation: 0 deg maps to max_t, 90 deg maps to min_t
  return max_t - ((max_t - min_t) * (abs_angle / 90.0));
}

// --- NEW: Temporal Smoother (Exponential Moving Average) ---
/**
 * @brief A Low-Pass Filter that remembers the target from the previous frame 
 * to prevent 30Hz high-frequency jitter caused by camera noise.
 */
class TargetSmoother
{
private:
  cv::Point2f prev_target_;
  bool initialized_;
  double alpha_;

public:
  // alpha = 0.3 means the final point is 30% new image data, 70% historical data.
  // Lower alpha = smoother trajectory but higher latency.
  TargetSmoother(double alpha = 0.3) : initialized_(false), alpha_(alpha) {}

  cv::Point smooth(const cv::Point & current_target)
  {
    if (!initialized_) {
      prev_target_ = current_target;
      initialized_ = true;
      return current_target;
    }

    // EMA Formula: S_t = a * Y_t + (1 - a) * S_{t-1}
    prev_target_.x = (alpha_ * current_target.x) + ((1.0 - alpha_) * prev_target_.x);
    prev_target_.y = (alpha_ * current_target.y) + ((1.0 - alpha_) * prev_target_.y);

    return cv::Point(std::round(prev_target_.x), std::round(prev_target_.y));
  }

  // Call this if the AUV loses the pipe completely to prevent dragging old data
  void reset() { initialized_ = false; }
};

}  // namespace trajectory_opt