#include "roadster_stunt/RoadsterDynamics.h"

#include <cmath>

namespace roadster_stunt {

namespace {
constexpr ocs2::scalar_t KILO = 1e3;  // inputs are in kN
}  // namespace

std::pair<ocs2::scalar_t, ocs2::scalar_t> RoadsterDynamics::reducedWrench(const ocs2::vector_t& input, ocs2::scalar_t jetY) {
  const ocs2::scalar_t Fz = KILO * (input(LIFT_L) + input(LIFT_R) - input(ROOF_L) - input(ROOF_R));
  const ocs2::scalar_t tauX = KILO * jetY * (input(LIFT_L) - input(LIFT_R) + input(ROOF_R) - input(ROOF_L));
  return {Fz, tauX};
}

ocs2::vector_t RoadsterDynamics::computeFlowMap(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
                                                const ocs2::PreComputation&) {
  const auto [Fz, tauX] = reducedWrench(input, param_.jetY_);
  const ocs2::scalar_t s = std::sin(state(ROLL));
  const ocs2::scalar_t c = std::cos(state(ROLL));

  ocs2::vector_t stateDerivative(STATE_DIM);
  stateDerivative(Y) = state(VY);
  stateDerivative(Z) = state(VZ);
  stateDerivative(VY) = -s * Fz / param_.mass_;
  stateDerivative(VZ) = c * Fz / param_.mass_ - param_.gravity_;
  stateDerivative(ROLL) = state(ROLL_RATE);
  stateDerivative(ROLL_RATE) = tauX / param_.rollInertia_;
  return stateDerivative;
}

ocs2::VectorFunctionLinearApproximation RoadsterDynamics::linearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                              const ocs2::vector_t& input,
                                                                              const ocs2::PreComputation& preComp) {
  ocs2::VectorFunctionLinearApproximation dynamics;
  dynamics.f = computeFlowMap(time, state, input, preComp);

  const auto [Fz, tauX] = reducedWrench(input, param_.jetY_);
  const ocs2::scalar_t s = std::sin(state(ROLL));
  const ocs2::scalar_t c = std::cos(state(ROLL));
  const ocs2::scalar_t invMass = 1.0 / param_.mass_;

  // Derivative with respect to state
  ocs2::matrix_t& A = dynamics.dfdx;
  A.setZero(STATE_DIM, STATE_DIM);
  A(Y, VY) = 1.0;
  A(Z, VZ) = 1.0;
  A(VY, ROLL) = -c * Fz * invMass;
  A(VZ, ROLL) = -s * Fz * invMass;
  A(ROLL, ROLL_RATE) = 1.0;

  // Derivative with respect to input: dFz/du = KILO * [1, 1, -1, -1], dtauX/du = KILO * jetY * [1, -1, -1, 1]
  ocs2::vector_t dFz(INPUT_DIM), dTau(INPUT_DIM);
  dFz << 1.0, 1.0, -1.0, -1.0;
  dTau << 1.0, -1.0, -1.0, 1.0;
  dFz *= KILO;
  dTau *= KILO * param_.jetY_;

  ocs2::matrix_t& B = dynamics.dfdu;
  B.setZero(STATE_DIM, INPUT_DIM);
  B.row(VY) = -s * invMass * dFz.transpose();
  B.row(VZ) = c * invMass * dFz.transpose();
  B.row(ROLL_RATE) = dTau.transpose() / param_.rollInertia_;

  return dynamics;
}

}  // namespace roadster_stunt
