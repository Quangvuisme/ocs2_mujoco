#pragma once

#include <Eigen/Dense>

#include <ocs2_core/Types.h>

#include "roadster_stunt/RoadsterDefinition.h"
#include "roadster_stunt/RoadsterParameters.h"

namespace roadster_stunt {

using Vector6 = Eigen::Matrix<ocs2::scalar_t, 6, 1>;
using Matrix3 = Eigen::Matrix<ocs2::scalar_t, 3, 3>;
using Vector3 = Eigen::Matrix<ocs2::scalar_t, 3, 1>;

/**
 * Turns the NMPC's four jet-group forces into the twelve on/off jet commands.
 *
 *  1. wrench: body z force and roll torque from the NMPC groups, plus pitch and yaw torques from a PD that
 *     keeps the car square to the road (the planar NMPC model does not see pitch or yaw);
 *  2. allocation: bounded least squares  min |W (A f - w)|^2 + reg |f|^2,  0 <= f <= f_max,
 *     solved by projected Gauss-Seidel (12 variables, warm started);
 *  3. valves: cold-gas jets are on/off, so each jet is modulated by a first-order sigma-delta whose
 *     on/off output averages to the commanded force.
 */
class JetController {
 public:
  struct Settings {
    ocs2::scalar_t pitchNaturalFrequency = 6.0;  // [rad/s]
    ocs2::scalar_t yawNaturalFrequency = 4.0;    // [rad/s]
    Vector6 wrenchWeights = (Vector6() << 0.0, 0.2, 1.0, 1.0, 1.0, 1.0).finished();  // Fx Fy Fz Tx Ty Tz
    ocs2::scalar_t regularization = 1e-3;
    size_t sweeps = 60;
    bool onOffValves = true;
  };

  /**
   * @param allocation: 6 x NUM_JETS map from jet forces [N] to the body wrench [F; tau] about the CoM (body frame).
   * @param fMax: NUM_JETS maximum jet forces [N].
   */
  JetController(Eigen::MatrixXd allocation, ocs2::vector_t fMax, RoadsterParameters roadster, Settings settings);

  /**
   * @param groupForces: NMPC input [lift_L, lift_R, roof_L, roof_R] in kN.
   * @param R: body orientation (world from body).
   * @param omegaBody: body angular velocity [rad/s].
   * @return jet forces [N] to apply until the next control tick.
   */
  ocs2::vector_t compute(const ocs2::vector_t& groupForces, const Matrix3& R, const Vector3& omegaBody);

  /** Bounded least-squares allocation of a body wrench [N, N m] to the jets [N]. */
  ocs2::vector_t allocate(const Vector6& wrench);

  /** Roll angle of the body z axis in the world y-z plane (same convention as the NMPC state). */
  static ocs2::scalar_t rollAngle(const Matrix3& R) { return std::atan2(-R(1, 2), R(2, 2)); }

  void reset();

 private:
  ocs2::vector_t modulate(const ocs2::vector_t& forces);

  Eigen::MatrixXd A_;  // [kN, kN m] per kN
  ocs2::vector_t fMax_;  // [kN]
  RoadsterParameters roadster_;
  Settings settings_;

  Eigen::MatrixXd H_;  // A^T W A + reg I
  Eigen::MatrixXd AtW_;
  ocs2::vector_t f_;  // allocation warm start [kN]
  ocs2::vector_t sigmaDelta_;  // valve accumulators [kN]
};

/** Remaining total impulse of the air tank; scales the jets down once it is empty. */
class AirTank {
 public:
  explicit AirTank(ocs2::scalar_t budget) : budget_(budget), remaining_(budget) {}

  ocs2::vector_t draw(const ocs2::vector_t& forces, ocs2::scalar_t dt);

  ocs2::scalar_t remaining() const { return remaining_; }
  ocs2::scalar_t fraction() const { return remaining_ / budget_; }
  void reset() { remaining_ = budget_; }

 private:
  ocs2::scalar_t budget_;
  ocs2::scalar_t remaining_;
};

}  // namespace roadster_stunt
