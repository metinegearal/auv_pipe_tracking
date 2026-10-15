#pragma once

// C++ port of NBV_paper_ros2/motion_control/{dlqr,thrusters}.py.
// State: [vx, vy, vz, px, py, pz] in a z-up world frame. Forces are newtons.
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pipe_track_dlqr
{
using State = Eigen::Matrix<double, 6, 1>;
using Thrusters = Eigen::Matrix<double, 8, 1>;
using WrenchMap = Eigen::Matrix<double, 6, 8>;

struct Settings
{
  double dt = 1.0 / 30.0;
  double mass = 31.02;
  double damping = 3.0; // it cant be used in the real world, but it is used in the simulation to make the vehicle more stable
  bool perfect = true;
  State state_weights = (State() << 800., 800., 800., 160., 160., 160.).finished();
  Eigen::Vector3d velocity_weights = Eigen::Vector3d::Constant(10.);
  Eigen::Vector3d force_weights = Eigen::Vector3d::Constant(0.1);
};

class FeedbackSystem
{
public:
  explicit FeedbackSystem(const Settings & settings = Settings())
  : settings_(settings)
  {
    if (!std::isfinite(settings.dt) || settings.dt <= 0. ||
      !std::isfinite(settings.mass) || settings.mass <= 0. ||
      !std::isfinite(settings.damping) || settings.damping < 0. ||
      !settings.state_weights.allFinite() || (settings.state_weights.array() <= 0.).any() ||
      !settings.velocity_weights.allFinite() || (settings.velocity_weights.array() <= 0.).any() ||
      !settings.force_weights.allFinite() || (settings.force_weights.array() <= 0.).any())
    {
      throw std::invalid_argument("Invalid DLQR plant or weights");
    }
    const double rate_dt = settings.damping * settings.dt / settings.mass;
    const double decay = std::exp(-rate_dt);
    double velocity_integral, position_integral;
    if (std::abs(rate_dt) < 1e-4) {
      velocity_integral = settings.dt *
        (1. - rate_dt / 2. + rate_dt * rate_dt / 6. - std::pow(rate_dt, 3) / 24.);
      position_integral = settings.dt * settings.dt *
        (0.5 - rate_dt / 6. + rate_dt * rate_dt / 24. - std::pow(rate_dt, 3) / 120.);
    } else {
      velocity_integral = -std::expm1(-rate_dt) * settings.dt / rate_dt;
      position_integral = (settings.dt - velocity_integral) * settings.dt / rate_dt;
    }
    a_.setIdentity();
    a_.topLeftCorner<3, 3>() *= decay;
    a_.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * velocity_integral;
    b_.topRows<3>() = Eigen::Matrix3d::Identity() * velocity_integral / settings.mass;
    b_.bottomRows<3>() = Eigen::Matrix3d::Identity() * position_integral / settings.mass;
    buoyancy_ = Eigen::Vector3d(0., 0., settings.perfect ? 0. :
        (997. * 0.03554577 - settings.mass) * 9.81);

    // The isotropic model separates into three independent 2-state DAREs.
    state_gain_.setZero();
    velocity_gain_.setZero();
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Matrix2d a;
      a << decay, 0., velocity_integral, 1.;
      Eigen::Vector2d b(b_(axis, axis), b_(axis + 3, axis));
      Eigen::Matrix2d q = Eigen::Matrix2d::Zero();
      q.diagonal() << settings.state_weights[axis], settings.state_weights[axis + 3];
      const double r = settings.force_weights[axis];
      Eigen::Matrix2d p = q;
      bool converged = false;
      //The Riccati Iteration
      for (int iteration = 0; iteration < 100000; ++iteration) {
        const Eigen::RowVector2d gain = (b.transpose() * p * a) / (r + b.dot(p * b));
        Eigen::Matrix2d next = q + a.transpose() * p * a - a.transpose() * p * b * gain;
        next = (0.5 * (next + next.transpose())).eval();
        if (!next.allFinite()) {throw std::runtime_error("DLQR Riccati iteration diverged");}
        converged = (next - p).norm() <= 1e-12 * std::max(1., next.norm());
        p = next;
        if (converged) {break;}
      }
      if (!converged) {throw std::runtime_error("DLQR Riccati iteration did not converge");}
      const Eigen::RowVector2d gain = (b.transpose() * p * a) / (r + b.dot(p * b));
      state_gain_(axis, axis) = gain[0];
      state_gain_(axis, axis + 3) = gain[1];

      // Positive stabilizing solution of the scalar velocity DARE.
      const double bv = b_(axis, axis);
      const double qv = settings.velocity_weights[axis];
      const double c = r * (1. - decay * decay) - qv * bv * bv;
      const double root = std::sqrt(c * c + 4. * bv * bv * qv * r);
      const double pv = c >= 0. ? 2. * qv * r / (c + root) :
        (-c + root) / (2. * bv * bv);
      velocity_gain_(axis, axis) = bv * pv * decay / (r + bv * bv * pv);
    }
  }

  Eigen::Vector3d state_force(const State & state, const State & reference) const
  {
    require_finite(state);
    require_finite(reference);
    return state_gain_ * (reference - state) +
           settings_.damping * reference.head<3>() - buoyancy_;
  }

  Eigen::Vector3d velocity_force(const State & state, const Eigen::Vector3d & reference) const
  {
    require_finite(state);
    require_finite(reference);
    return velocity_gain_ * (reference - state.head<3>()) +
           settings_.damping * reference - buoyancy_;
  }

  State predict(const State & state, const Eigen::Vector3d & force) const
  {
    require_finite(state);
    require_finite(force);
    return a_ * state + b_ * (force + buoyancy_);
  }

  const Eigen::Matrix<double, 6, 6> & a() const {return a_;}
  const Eigen::Matrix<double, 6, 3> & b() const {return b_;}
  const Eigen::Matrix<double, 3, 6> & state_gain() const {return state_gain_;}
  const Eigen::Matrix3d & velocity_gain() const {return velocity_gain_;}

private:
  template<typename Derived>
  static void require_finite(const Eigen::MatrixBase<Derived> & value)
  {
    if (!value.allFinite()) {throw std::invalid_argument("DLQR inputs must be finite");}
  }
  Settings settings_;
  Eigen::Matrix<double, 6, 6> a_;
  Eigen::Matrix<double, 6, 3> b_;
  Eigen::Matrix<double, 3, 6> state_gain_;
  Eigen::Matrix3d velocity_gain_;
  Eigen::Vector3d buoyancy_;
};

inline Eigen::Vector3d waypoint_velocity(
  const Eigen::Vector3d & position, const Eigen::Vector3d & target,
  double speed, double braking_acceleration, double stopping_distance)
{
  if (!position.allFinite() || !target.allFinite() || !std::isfinite(speed) || speed < 0. ||
    !std::isfinite(braking_acceleration) || braking_acceleration <= 0. ||
    !std::isfinite(stopping_distance) || stopping_distance < 0.)
  {
    throw std::invalid_argument("Invalid waypoint velocity inputs");
  }
  const Eigen::Vector3d delta = target - position;
  const double distance = delta.norm();
  if (distance < 1e-9) {return Eigen::Vector3d::Zero();}
  const double requested_speed = std::min(speed,
    std::sqrt(2. * braking_acceleration * std::max(0., distance - stopping_distance)));
  return delta * (requested_speed / distance);
}

class ThrusterAllocator
{
public:
  static constexpr double max_thrust = 77.55;

  explicit ThrusterAllocator(bool perfect = true)
  {
    // HoveringAUV geometry, converted from Unreal cm to client metres.
    Eigen::Matrix<double, 8, 3> mesh;
    mesh << 18.18, 22.14, -4., 18.18, -22.14, -4.,
      -31.43, -22.14, -4., -31.43, 22.14, -4.,
      7.39, 18.23, -0.21, 7.39, -18.23, -0.21,
      -20.64, -18.23, -0.21, -20.64, 18.23, -0.21;
    Eigen::Vector3d center(-5.9, 0.46, -2.82);
    if (perfect) {
      center = (mesh.row(0) + mesh.row(2)).transpose() / 2.;
      center.z() = mesh(7, 2);
    }
    const double diagonal = 1. / std::sqrt(2.);
    for (int i = 0; i < 8; ++i) {
      Eigen::Vector3d position = (mesh.row(i).transpose() - center).cwiseProduct(
        Eigen::Vector3d(0.01, -0.01, 0.01));
      Eigen::Vector3d direction(0., 0., 1.);
      if (i >= 4) {direction = Eigen::Vector3d(diagonal, i % 2 ? -diagonal : diagonal, 0.);}
      mapping_.col(i).head<3>() = direction;
      mapping_.col(i).tail<3>() = position.cross(direction);
    }
    const Eigen::Matrix<double, 6, 6> gram = mapping_ * mapping_.transpose();
    // Robust Cholesky Decomposition
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(gram);
    if (solver.info() != Eigen::Success || !solver.isPositive()) {
      throw std::runtime_error("Thruster mapping is singular");
    }
    allocation_ = mapping_.transpose() * solver.solve(Eigen::Matrix<double, 6, 6>::Identity());
  }

  Thrusters allocate(const State & body_wrench) const
  {
    if (!body_wrench.allFinite()) {throw std::invalid_argument("Wrench must be finite");}
    return allocation_ * body_wrench;
  }

  const WrenchMap & mapping() const {return mapping_;}

private:
  WrenchMap mapping_;
  Eigen::Matrix<double, 8, 6> allocation_;
};
}  // namespace pipe_track_dlqr
