#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "utils/map_target.hpp"

namespace
{
void expect_point(const cv::Point2f & actual, const cv::Point2f & expected, const char * label)
{
  if (cv::norm(actual - expected) > 1e-5) {
    throw std::runtime_error(std::string(label) + ": unexpected target (" +
      std::to_string(actual.x) + ", " + std::to_string(actual.y) + ")");
  }
}
}  // namespace

int main()
{
  try {
    map_navigation::TargetSelector selector;
    const cv::Point2f pose(38.974f, -54.727f);

    // With no map yet, hold the measured position exactly, including sub-cell precision.
    expect_point(selector.select({}, pose, 0.25), pose, "initial empty map");

    // Fixture based on the logged pose/target. The old mixed-unit comparison picks
    // (119,-203), publishing (29.75,-50.75), although (39,-55) is much closer.
    const std::vector<cv::Point2f> frontiers{{119, -203}, {156, -220}};
    expect_point(selector.select(frontiers, pose, 0.25), {39, -55}, "nearest in metres");

    // Existing EMA operates in cells: midpoint between (156,-220) and (160,-216).
    expect_point(selector.select({{160, -216}}, pose, 0.25), {39.5f, -54.5f}, "cell smoothing");

    // Exhausting the map must not scale or smooth the current vehicle position.
    const cv::Point2f moved_pose(38.958f, -54.703f);
    expect_point(selector.select({}, moved_pose, 0.25), moved_pose, "finished map hold");
    expect_point(selector.select({{120, -200}}, moved_pose, 0.25), {30, -50}, "smoother reset");

    // Nearest-point selection must not depend on the arbitrary map origin.
    map_navigation::TargetSelector shifted;
    expect_point(shifted.select({{19, -3}, {56, -20}}, pose + cv::Point2f(-25, 50), 0.25),
      {14, -5}, "translated map origin");

    map_navigation::TargetSelector different_resolution;
    expect_point(different_resolution.select({{20, -20}, {4, -4}}, {9.9f, -9.9f}, 0.5),
      {10, -10}, "nondefault resolution");

    map_navigation::TargetSelector origin;
    expect_point(origin.select({}, {0, 0}, 0.25), {0, 0}, "zero position hold");
    expect_point(origin.select({{-2, 0}, {2, 0}}, {0, 0}, 0.25), {-0.5f, 0}, "stable tie");
    std::cout << "PASS: 9 map-target coordinate and smoothing regression checks\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
