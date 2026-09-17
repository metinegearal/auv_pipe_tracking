#include "pipe_track_estimation/gtsam_estimator.hpp"
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::X; // Pose
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::B; // Bias

namespace pipe_track_estimation {

GtsamEstimator::GtsamEstimator() : state_index_(0) {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = std::make_shared<gtsam::ISAM2>(parameters);

    // --- ALIGNED WITH HOLOOCEAN JSON ---
    double accel_sigma = 0.03;      // From JSON: AccelSigma
    double gyro_sigma = 0.0035;     // From JSON: AngVelSigma
    double dvl_sigma = 0.001;       // From JSON: VelSigma
    double mag_yaw_sigma = 0.05;    // From your Python EKF (0.05**2)

    prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
    prior_vel_noise_  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.01, 0.01));
    prior_bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
    
    dvl_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(dvl_sigma, dvl_sigma, dvl_sigma));
    
    // Yaw-Only Noise (Ignore Roll, Pitch, X, Y, Z. Trust Yaw)
    mag_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e5, 1e5, mag_yaw_sigma, 1e5, 1e5, 1e5).finished()
    );
}

void GtsamEstimator::initialize(const gtsam::Pose3& initial_pose, 
                                const gtsam::Vector3& initial_velocity, 
                                const gtsam::imuBias::ConstantBias& initial_bias) 
{
    std::lock_guard<std::mutex> lock(graph_mutex_);

    // 1. Add Prior Factors (Anchor the start point)
    graph_.addPrior(X(0), initial_pose, prior_pose_noise_);
    graph_.addPrior(V(0), initial_velocity, prior_vel_noise_);
    graph_.addPrior(B(0), initial_bias, prior_bias_noise_);

    // 2. Add Initial Estimates
    initial_estimates_.insert(X(0), initial_pose);
    initial_estimates_.insert(V(0), initial_velocity);
    initial_estimates_.insert(B(0), initial_bias);

    // 3. Setup IMU Preintegration Parameters
    auto preintegration_params = gtsam::PreintegrationParams::MakeSharedU(9.81);
    preintegration_params->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(0.01, 2);
    preintegration_params->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(0.001, 2);
    preintegration_params->integrationCovariance = gtsam::Matrix33::Identity(3,3) * pow(0.0001, 2);
    
    imu_preintegrated_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(preintegration_params, initial_bias);

    // 4. Optimize the initial graph
    isam_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    current_pose_ = initial_pose;
    current_velocity_ = initial_velocity;
    current_bias_ = initial_bias;
}

void GtsamEstimator::add_imu_measurement(const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel, double dt) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    // Accumulate the high-frequency IMU data mathematically without adding to the graph yet
    imu_preintegrated_->integrateMeasurement(linear_accel, angular_vel, dt);
}

void GtsamEstimator::add_vo_measurement(const gtsam::Pose3& vo_pose, double /*timestamp*/) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    // --- NEW SAFETY CHECK ---
    if (state_index_ > 0 && imu_preintegrated_->deltaTij() < 1e-4) {
        return; // Skip if dt is practically zero
    }
    // ------------------------

    state_index_++;

    // 1. Add the accumulated IMU factor connecting previous state to this new state
    gtsam::ImuFactor imu_factor(X(state_index_ - 1), V(state_index_ - 1),
                                X(state_index_), V(state_index_),
                                B(state_index_ - 1), *imu_preintegrated_);
    graph_.add(imu_factor);

    // 2. Add Bias factor (assume bias doesn't change wildly between steps)
    graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(state_index_ - 1), B(state_index_), gtsam::imuBias::ConstantBias(), prior_bias_noise_));

    // 3. Add the VO Pose Factor
    graph_.addPrior(X(state_index_), vo_pose, vo_noise_);

    // 4. Predict new state for initialization
    gtsam::NavState prop_state = imu_preintegrated_->predict(gtsam::NavState(current_pose_, current_velocity_), current_bias_);
    initial_estimates_.insert(X(state_index_), prop_state.pose());
    initial_estimates_.insert(V(state_index_), prop_state.velocity());
    initial_estimates_.insert(B(state_index_), current_bias_);

    // 5. Optimize
    isam_->update(graph_, initial_estimates_);
    isam_->update(); // Double update for convergence
    
    graph_.resize(0);
    initial_estimates_.clear();

    // 6. Extract optimized state and reset preintegration
    current_pose_ = isam_->calculateEstimate<gtsam::Pose3>(X(state_index_));
    current_velocity_ = isam_->calculateEstimate<gtsam::Vector3>(V(state_index_));
    current_bias_ = isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(state_index_));
    
    imu_preintegrated_->resetIntegrationAndSetBias(current_bias_);
}

// void GtsamEstimator::add_dvl_measurement(const Eigen::Vector3d& linear_velocity_body, double /*timestamp*/) {
//     std::lock_guard<std::mutex> lock(graph_mutex_);
//     // if (state_index_ == 0) return; // Wait for initialization

//     if (imu_preintegrated_->deltaTij() < 1e-4) {
//         std::cout << "[WARNING] DVL dropped: No IMU data! (Check IMU topic)" << std::endl;
//         return; // Skip if dt is practically zero
//     }

//     state_index_++;

//     // 1. Add the accumulated IMU factor connecting previous state to this new state
//     gtsam::ImuFactor imu_factor(X(state_index_ - 1), V(state_index_ - 1),
//                                 X(state_index_), V(state_index_),
//                                 B(state_index_ - 1), *imu_preintegrated_);
//     graph_.add(imu_factor);

//     // 2. Add Bias factor
//     graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
//         B(state_index_ - 1), B(state_index_), gtsam::imuBias::ConstantBias(), prior_bias_noise_));

//     // 3. DVL Velocity Factor
//     // The DVL measures velocity relative to the AUV's body. GTSAM needs it in the world frame.
//     // We rotate the DVL body velocity into world velocity using our last known orientation.
//     gtsam::Vector3 world_velocity = current_pose_.rotation() * linear_velocity_body;
//     graph_.addPrior(V(state_index_), world_velocity, dvl_noise_);

//     // 4. Predict new state for initialization
//     gtsam::NavState prop_state = imu_preintegrated_->predict(gtsam::NavState(current_pose_, current_velocity_), current_bias_);
//     initial_estimates_.insert(X(state_index_), prop_state.pose());
//     initial_estimates_.insert(V(state_index_), prop_state.velocity());
//     initial_estimates_.insert(B(state_index_), current_bias_);

//     // 5. Optimize
//     isam_->update(graph_, initial_estimates_);
//     isam_->update(); 
    
//     graph_.resize(0);
//     initial_estimates_.clear();

//     // 6. Extract optimized state and reset preintegration
//     current_pose_ = isam_->calculateEstimate<gtsam::Pose3>(X(state_index_));
//     current_velocity_ = isam_->calculateEstimate<gtsam::Vector3>(V(state_index_));
//     current_bias_ = isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(state_index_));
    
//     imu_preintegrated_->resetIntegrationAndSetBias(current_bias_);
// }


void GtsamEstimator::add_dvl_measurement(const Eigen::Vector3d& linear_velocity_body, double /*timestamp*/, double absolute_yaw) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    
    // if (state_index_ == 0) return;
    if (imu_preintegrated_->deltaTij() < 1e-4) return;

    state_index_++;

    // 1. IMU Factor
    gtsam::ImuFactor imu_factor(X(state_index_ - 1), V(state_index_ - 1),
                                X(state_index_), V(state_index_),
                                B(state_index_ - 1), *imu_preintegrated_);
    graph_.add(imu_factor);

    // 2. Bias Factor
    graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(state_index_ - 1), B(state_index_), gtsam::imuBias::ConstantBias(), prior_bias_noise_));

    // 3. DVL Factor
    gtsam::Vector3 world_velocity = current_pose_.rotation() * linear_velocity_body;
    graph_.addPrior(V(state_index_), world_velocity, dvl_noise_);

    // --- NEW: MAGNETOMETER YAW FACTOR ---
    // We create a dummy Pose3, but because of our mag_noise_ mask, GTSAM only looks at the Yaw!
    gtsam::Pose3 yaw_constraint(gtsam::Rot3::Yaw(absolute_yaw), gtsam::Point3(0,0,0));
    graph_.addPrior(X(state_index_), yaw_constraint, mag_noise_);
    // ------------------------------------

    // 4. Predict & Optimize
    gtsam::NavState prop_state = imu_preintegrated_->predict(gtsam::NavState(current_pose_, current_velocity_), current_bias_);
    initial_estimates_.insert(X(state_index_), prop_state.pose());
    initial_estimates_.insert(V(state_index_), prop_state.velocity());
    initial_estimates_.insert(B(state_index_), current_bias_);

    isam_->update(graph_, initial_estimates_);
    isam_->update(); 
    
    graph_.resize(0);
    initial_estimates_.clear();

    current_pose_ = isam_->calculateEstimate<gtsam::Pose3>(X(state_index_));
    current_velocity_ = isam_->calculateEstimate<gtsam::Vector3>(V(state_index_));
    current_bias_ = isam_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(state_index_));
    
    imu_preintegrated_->resetIntegrationAndSetBias(current_bias_);
}

gtsam::Pose3 GtsamEstimator::get_current_pose() const { return current_pose_; }
gtsam::Vector3 GtsamEstimator::get_current_velocity() const { return current_velocity_; }

} // namespace pipe_track_estimation