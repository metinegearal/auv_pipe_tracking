#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace math_utils
{

inline std::vector<double> rotate_point_2d(double x, double y, double yaw_angle = 2.51)
{
  double rotatedX = x * std::cos(yaw_angle) - y * std::sin(yaw_angle);
  double rotatedY = x * std::sin(yaw_angle) + y * std::cos(yaw_angle);
  return {rotatedX, rotatedY};
}

inline double calculate_angle_from_center_to_pixel(
  double px, double fov = 90.0, double width = 288.0)
{
  double fx = (width / 2.0) / std::tan((fov / 2.0) * M_PI / 180.0);
  double cx = width / 2.0;
  return std::atan((px - cx) / fx);
}

inline std::vector<double> point_from_depth(
  const std::vector<double> & center, double yaw, double depth, bool downcamera = false)
{
  double angle_x_local = -calculate_angle_from_center_to_pixel(center[0]);
  double angle_y_local = -calculate_angle_from_center_to_pixel(center[1]);

  double x, y, z;
  if (downcamera) {
    double dist_x_local = depth * std::tan(angle_x_local);
    double dist_y_local = depth * std::tan(angle_y_local);

    x = dist_y_local * std::cos(yaw) - dist_x_local * std::sin(yaw);
    y = dist_y_local * std::sin(yaw) + dist_x_local * std::cos(yaw);
    z = depth;
  } else {
    double angle_x_global = angle_x_local + yaw;
    x = depth * std::cos(angle_y_local) * std::cos(angle_x_global);
    y = depth * std::cos(angle_y_local) * std::sin(angle_x_global);
    z = depth * std::sin(angle_y_local);
  }
  return {x, y, z};
}

inline Eigen::Vector3d point_from_depth_matrix(
  const Eigen::Vector2d & pixel, double depth, const Eigen::Matrix3d & K,
  const Eigen::Matrix4d & T_world_cam)
{
  // 1. Unproject pixel to 3D point in the local camera frame
  Eigen::Vector3d pt_cam;
  pt_cam.z() = depth;
  pt_cam.x() = (pixel.x() - K(0, 2)) * depth / K(0, 0);  // (u - cx) * Z / fx
  pt_cam.y() = (pixel.y() - K(1, 2)) * depth / K(1, 1);  // (v - cy) * Z / fy

  // 2. Convert to homogeneous coordinates (add a 1.0 at the end)
  Eigen::Vector4d pt_cam_homo;
  pt_cam_homo << pt_cam, 1.0;

  // 3. Transform to world coordinates using the SE(3) matrix
  Eigen::Vector4d pt_world_homo = T_world_cam * pt_cam_homo;

  // 4. Return the x, y, z components
  return pt_world_homo.head<3>();
}

inline std::vector<double> angle_to_waypoint(double angle_rad, double distance = 5.0)
{
  return {distance * std::cos(angle_rad), distance * std::sin(angle_rad)};
}

}  // namespace math_utils