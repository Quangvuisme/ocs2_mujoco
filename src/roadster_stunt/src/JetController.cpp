#include "roadster_stunt/JetController.h"

#include <algorithm>
#include <cmath>

#include "roadster_stunt/RoadsterDynamics.h"

namespace roadster_stunt {

namespace {
constexpr ocs2::scalar_t KILO = 1e3;
}  // namespace

JetController::JetController(Eigen::MatrixXd allocation, ocs2::vector_t fMax, RoadsterParameters roadster, Settings settings)
    : A_(std::move(allocation)), fMax_(fMax / KILO), roadster_(std::move(roadster)), settings_(std::move(settings)) {
  const Eigen::MatrixXd W = settings_.wrenchWeights.asDiagonal();
  AtW_ = A_.transpose() * W;
  H_ = AtW_ * A_ + settings_.regularization * Eigen::MatrixXd::Identity(A_.cols(), A_.cols());
  reset();
}

void JetController::reset() {
  f_ = ocs2::vector_t::Zero(A_.cols());
  sigmaDelta_ = ocs2::vector_t::Zero(A_.cols());
}

ocs2::vector_t JetController::compute(const ocs2::vector_t& groupForces, const Matrix3& R, const Vector3& omegaBody) {
  // roll axis: straight from the NMPC
  const auto [Fz, tauX] = RoadsterDynamics::reducedWrench(groupForces, roadster_.jetY_);

  // pitch and yaw: PD towards the pure-roll attitude Rx(roll) (LQR gains of a double integrator per axis)
  const ocs2::scalar_t roll = rollAngle(R);
  const Matrix3 Rdes = Eigen::AngleAxis<ocs2::scalar_t>(roll, Vector3::UnitX()).toRotationMatrix();
  const Matrix3 E = Rdes.transpose() * R - R.transpose() * Rdes;
  const Vector3 eR(0.5 * E(2, 1), 0.5 * E(0, 2), 0.5 * E(1, 0));
  const ocs2::scalar_t wp = settings_.pitchNaturalFrequency;
  const ocs2::scalar_t wy = settings_.yawNaturalFrequency;
  const ocs2::scalar_t tauY = -roadster_.pitchInertia_ * (wp * wp * eR(1) + std::sqrt(2.0) * wp * omegaBody(1));
  const ocs2::scalar_t tauZ = -roadster_.yawInertia_ * (wy * wy * eR(2) + std::sqrt(2.0) * wy * omegaBody(2));

  Vector6 wrench;
  wrench << 0.0, 0.0, Fz, tauX, tauY, tauZ;
  const ocs2::vector_t forces = allocate(wrench);
  return settings_.onOffValves ? modulate(forces) : forces;
}

ocs2::vector_t JetController::allocate(const Vector6& wrench) {
  const ocs2::vector_t b = AtW_ * (wrench / KILO);
  for (size_t sweep = 0; sweep < settings_.sweeps; ++sweep) {
    for (Eigen::Index j = 0; j < f_.size(); ++j) {
      const ocs2::scalar_t residual = b(j) - H_.row(j).dot(f_) + H_(j, j) * f_(j);
      f_(j) = std::clamp(residual / H_(j, j), 0.0, fMax_(j));
    }
  }
  return KILO * f_;
}

ocs2::vector_t JetController::modulate(const ocs2::vector_t& forces) {
  ocs2::vector_t out(forces.size());
  for (Eigen::Index j = 0; j < forces.size(); ++j) {
    sigmaDelta_(j) += std::clamp(forces(j) / KILO, 0.0, fMax_(j));
    const bool open = sigmaDelta_(j) >= 0.5 * fMax_(j);
    out(j) = open ? fMax_(j) : 0.0;
    sigmaDelta_(j) = std::clamp(sigmaDelta_(j) - out(j), -fMax_(j), fMax_(j));
  }
  return KILO * out;
}

ocs2::vector_t AirTank::draw(const ocs2::vector_t& forces, ocs2::scalar_t dt) {
  const ocs2::scalar_t need = forces.sum() * dt;
  if (need <= remaining_) {
    remaining_ -= need;
    return forces;
  }
  const ocs2::scalar_t scale = need > 0.0 ? remaining_ / need : 0.0;
  remaining_ = 0.0;
  return scale * forces;
}

}  // namespace roadster_stunt
