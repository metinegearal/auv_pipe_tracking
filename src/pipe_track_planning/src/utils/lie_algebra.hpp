// include/auv_geometry/lie_algebra.hpp
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace auv_geometry {

constexpr double EPSILON = 1e-6;

// Skew-symmetric hat operator for so(3)
inline Eigen::Matrix3d hat(const Eigen::Vector3d& phi) {
    Eigen::Matrix3d S;
    S <<       0.0, -phi.z(),  phi.y(),
          phi.z(),       0.0, -phi.x(),
         -phi.y(),  phi.x(),       0.0;
    return S;
}

// Vee operator for so(3)
inline Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
    return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

// SO(3) Exponential Map: so(3) -> SO(3)
inline Eigen::Matrix3d exp_so3(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    Eigen::Matrix3d phi_hat = hat(phi);

    if (theta < EPSILON) {
        return Eigen::Matrix3d::Identity() + phi_hat;
    }

    double a = std::sin(theta) / theta;
    double b = (1.0 - std::cos(theta)) / (theta * theta);
    return Eigen::Matrix3d::Identity() + a * phi_hat + b * (phi_hat * phi_hat);
}

// SO(3) Logarithmic Map: SO(3) -> so(3)
inline Eigen::Vector3d log_so3(const Eigen::Matrix3d& R) {
    double tr = R.trace();
    double cos_theta = std::clamp((tr - 1.0) * 0.5, -1.0, 1.0);
    double theta = std::acos(cos_theta);

    if (theta < EPSILON) {
        return 0.5 * vee(R - R.transpose());
    }

    return (theta / (2.0 * std::sin(theta))) * vee(R - R.transpose());
}

// Left Jacobian of SO(3)
inline Eigen::Matrix3d left_jacobian(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    Eigen::Matrix3d phi_hat = hat(phi);

    if (theta < EPSILON) {
        return Eigen::Matrix3d::Identity() + 0.5 * phi_hat;
    }

    double a = (1.0 - std::cos(theta)) / (theta * theta);
    double b = (theta - std::sin(theta)) / (theta * theta * theta);
    return Eigen::Matrix3d::Identity() + a * phi_hat + b * (phi_hat * phi_hat);
}

// Inverse of Left Jacobian of SO(3)
inline Eigen::Matrix3d left_jacobian_inv(const Eigen::Vector3d& phi) {
    double theta = phi.norm();
    Eigen::Matrix3d phi_hat = hat(phi);

    if (theta < EPSILON) {
        return Eigen::Matrix3d::Identity() - 0.5 * phi_hat + (1.0 / 12.0) * (phi_hat * phi_hat);
    }

    double c = (1.0 / (theta * theta)) - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
    return Eigen::Matrix3d::Identity() - 0.5 * phi_hat + c * (phi_hat * phi_hat);
}

// SE(3) Exponential Map: se(3) (phi, rho) -> 4x4 Transformation Matrix
inline Eigen::Matrix4d exp_se3(const Eigen::Matrix<double, 6, 1>& xi) {
    Eigen::Vector3d phi = xi.head<3>();
    Eigen::Vector3d rho = xi.tail<3>();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = exp_so3(phi);
    T.block<3, 1>(0, 3) = left_jacobian(phi) * rho;
    return T;
}

// SE(3) Logarithmic Map: 4x4 Matrix -> se(3) (phi, rho)
inline Eigen::Matrix<double, 6, 1> log_se3(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    Eigen::Vector3d phi = log_so3(R);
    Eigen::Vector3d rho = left_jacobian_inv(phi) * t;

    Eigen::Matrix<double, 6, 1> xi;
    xi.head<3>() = phi;
    xi.tail<3>() = rho;
    return xi;
}

// Double Geodesic Distance in SE(3)
inline double double_geodesic_distance(const Eigen::Matrix4d& TA, const Eigen::Matrix4d& TB) {
    Eigen::Matrix3d RA = TA.block<3, 3>(0, 0);
    Eigen::Matrix3d RB = TB.block<3, 3>(0, 0);
    Eigen::Vector3d tA = TA.block<3, 1>(0, 3);
    Eigen::Vector3d tB = TB.block<3, 1>(0, 3);

    double rot_dist = log_so3(RA.transpose() * RB).norm();
    double trans_dist = (tB - tA).norm();

    return std::sqrt(rot_dist * rot_dist + trans_dist * trans_dist);
}

} // namespace auv_geometry