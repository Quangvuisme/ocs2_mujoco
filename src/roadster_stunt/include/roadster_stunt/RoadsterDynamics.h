#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include "roadster_stunt/RoadsterDefinition.h"
#include "roadster_stunt/RoadsterParameters.h"

namespace roadster_stunt {

/**
 * Planar roll dynamics of the flying car.
 *
 * The jets give a force along the body z axis and a torque about the body x (roll) axis:
 *   Fz   = lift_L + lift_R - roof_L - roof_R
 *   tauX = jetY * (lift_L - lift_R) + jetY * (roof_R - roof_L)
 * and the body z axis points along (-sin(roll), cos(roll)) in the y-z plane, so
 *   vy'  = -sin(roll) Fz / m,   vz' = cos(roll) Fz / m - g,   rollRate' = tauX / Ixx.
 */
class RoadsterDynamics final : public ocs2::SystemDynamicsBase {
 public:
  explicit RoadsterDynamics(RoadsterParameters parameters) : param_(std::move(parameters)) {}

  ~RoadsterDynamics() override = default;

  RoadsterDynamics* clone() const override { return new RoadsterDynamics(*this); }

  ocs2::vector_t computeFlowMap(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                const ocs2::PreComputation&) override;

  ocs2::VectorFunctionLinearApproximation linearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                              const ocs2::vector_t& input,
                                                              const ocs2::PreComputation&) override;

  /** Body z force [N] and roll torque [N m] of a (kN) input vector. */
  static std::pair<ocs2::scalar_t, ocs2::scalar_t> reducedWrench(const ocs2::vector_t& input, ocs2::scalar_t jetY);

 private:
  RoadsterParameters param_;
};

}  // namespace roadster_stunt
