#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include "ocs2_quadrotor_nmpc/QuadrotorParameters.h"
#include "ocs2_quadrotor_nmpc/QuadrotorDefinition.h"

namespace ocs2_cartpole_quadrotor {

/**
 * Quadrotor system dynamics (Analytical version - same as quadrotor_online)
 * State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
 * Input: [Fz, Mx, My, Mz] - Thrust force and moments
 */
class QuadrotorDynamics final : public ocs2::SystemDynamicsBase {
 public:
  explicit QuadrotorDynamics(QuadrotorParameters quadrotorParameters) 
      : param_(std::move(quadrotorParameters)) {}

  ~QuadrotorDynamics() override = default;

  QuadrotorDynamics* clone() const override { return new QuadrotorDynamics(*this); }

  ocs2::vector_t computeFlowMap(ocs2::scalar_t time, 
                                 const ocs2::vector_t& state, 
                                 const ocs2::vector_t& input, 
                                 const ocs2::PreComputation&) override;

  ocs2::VectorFunctionLinearApproximation linearApproximation(
      ocs2::scalar_t t, 
      const ocs2::vector_t& x, 
      const ocs2::vector_t& u, 
      const ocs2::PreComputation&) override;

 private:
  QuadrotorParameters param_;
  ocs2::matrix_t jacobianOfAngularVelocityMapping_;
};

}  // namespace ocs2_cartpole_quadrotor
