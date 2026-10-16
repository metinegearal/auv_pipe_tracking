#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "traj_opt.hpp"

namespace map_navigation
{
// Frontier points are map-cell indices. Positions and returned targets are metres.
class TargetSelector
{
public:
  cv::Point2f select(
    const std::vector<cv::Point2f> & frontier_cells, const cv::Point2f & position_world,
    double resolution)
  {
    if (frontier_cells.empty()) {
      smoother_.reset();
      return position_world;
    }

    cv::Point2f target_cell = frontier_cells.front();
    double closest_distance = cv::norm(target_cell * resolution - position_world);
    for (const auto & cell : frontier_cells) {
      const double distance = cv::norm(cell * resolution - position_world);
      if (distance < closest_distance) {
        target_cell = cell;
        closest_distance = distance;
      }
    }

    // Keep the existing smoothing in cell coordinates, preserving 0.25 m precision.
    const cv::Point smoothed_cell =
      smoother_.smooth(cv::Point(std::round(target_cell.x), std::round(target_cell.y)));
    return cv::Point2f(smoothed_cell.x * resolution, smoothed_cell.y * resolution);
  }

private:
  trajectory_opt::TargetSmoother smoother_{0.5};
};
}  // namespace map_navigation
