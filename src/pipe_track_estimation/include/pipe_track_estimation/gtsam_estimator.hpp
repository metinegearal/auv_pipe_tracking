#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>

namespace pipe_track_estimation
{

class GtsamEstimator
{
public:
  GtsamEstimator();
  ~GtsamEstimator() = default;

  // --- Initialization ---
  void initialize(
    const gtsam::Pose3 & initial_pose, const gtsam::Vector3 & initial_velocity,
    const gtsam::imuBias::ConstantBias & initial_bias);

  // --- Sensor Inputs ---
  // Feed high-rate IMU data here for preintegration
  void add_imu_measurement(
    const Eigen::Vector3d & linear_accel, const Eigen::Vector3d & angular_vel, double dt);

  // Add a DVL reading (Velocity constraint)
  void add_dvl_measurement(
    const Eigen::Vector3d & linear_velocity, double timestamp, double absolute_yaw);

  // Add Visual Odometry / Camera pose
  void add_vo_measurement(const gtsam::Pose3 & vo_pose, double timestamp);

  // --- Output ---
  // Extract the latest optimized state to send back to the ROS control loop
  gtsam::Pose3 get_current_pose() const;
  gtsam::Vector3 get_current_velocity() const;

private:
  std::mutex graph_mutex_;

  // GTSAM Graph and Optimizer
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  std::shared_ptr<gtsam::ISAM2> isam_;

  // IMU Preintegration Tool
  std::shared_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrated_;
  gtsam::noiseModel::Diagonal::shared_ptr mag_noise_;

  // State Tracking
  uint64_t state_index_;
  gtsam::Pose3 current_pose_;
  gtsam::Vector3 current_velocity_;
  gtsam::imuBias::ConstantBias current_bias_;

  // Noise Models (Covariance matrices)
  gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr dvl_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr vo_noise_;
};

}  // namespace pipe_track_estimation