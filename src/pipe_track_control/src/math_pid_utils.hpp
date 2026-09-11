#pragma once

#include <cmath>
#include <vector>
#include <array>

namespace utils {

    // --- Math Utilities ---
    inline double normalize_angle_deg(double angle) {
        return std::fmod(angle + 180.0, 360.0) - 180.0;
    }

    inline double quaternion_to_roll_deg(double x, double y, double z, double w) {
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        return std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
    }

    inline double quaternion_to_pitch_rad(double x, double y, double z, double w) {
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1.0) {
            return std::copysign(M_PI / 2.0, sinp);
        }
        return std::asin(sinp);
    }

    inline double diff_2d(const std::vector<double>& a, const std::vector<double>& b) {
        return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));
    }

    inline double diff_3d(const std::vector<double>& a, const std::vector<double>& b) {
        return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2));
    }

    inline std::vector<double> world_to_auv_coordinates(const std::vector<double>& world_pos, double yaw) {
        std::vector<double> auv_pos(3, 0.0);
        auv_pos[0] = std::cos(yaw) * world_pos[0] - std::sin(yaw) * world_pos[1];
        auv_pos[1] = std::sin(yaw) * world_pos[0] + std::cos(yaw) * world_pos[1];
        auv_pos[2] = world_pos[2];
        return auv_pos;
    }

    // --- PID Controller ---
    class PID {
    public:
        PID(double kp, double ki, double kd, double setpoint = 0.0, double filter_coeff = 0.2)
            : kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint), filter_coeff_(filter_coeff),
              integral_(0.0), prev_error_(0.0) {}

        double update(double measurement) {
            double error = setpoint_ - measurement;
            integral_ += error;
            double derivative = (error - prev_error_);
            
            // Apply simple low-pass filter to derivative if needed
            derivative = filter_coeff_ * derivative + (1.0 - filter_coeff_) * prev_derivative_;
            
            prev_error_ = error;
            prev_derivative_ = derivative;

            return kp_ * error + ki_ * integral_ + kd_ * derivative;
        }

    private:
        double kp_, ki_, kd_, setpoint_, filter_coeff_;
        double integral_;
        double prev_error_;
        double prev_derivative_ = 0.0;
    };

} // namespace utils