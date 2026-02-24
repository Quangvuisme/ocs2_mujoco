#include "ocs2_cartpole/CartpoleDynamicsPinocchio.h"

#include <iostream>
#include <stdexcept>

namespace ocs2_cartpole {

CartpoleDynamicsPinocchio::CartpoleDynamicsPinocchio(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const std::string& library_folder,
    bool verbose)
    : pinocchioInterfacePtr_(std::make_shared<ocs2::PinocchioInterface>(pinocchioInterface)) {
  
  if (verbose) {
    std::cerr << "[CartpoleDynamicsPinocchio] Initialized with PinocchioInterface\n";
  }

  // Initialize AutoDiff framework
  // State: [theta, x, theta_dot, x_dot] (4D)
  // Input: [force] (1D)
  initialize(4, 1, "cartpole_dynamics_pinocchio", library_folder, true, verbose);
}

CartpoleDynamicsPinocchio::CartpoleDynamicsPinocchio(
    const CartpoleDynamicsPinocchio& rhs)
    : ocs2::SystemDynamicsBaseAD(rhs),
      pinocchioInterfacePtr_(rhs.pinocchioInterfacePtr_) {}

ocs2::ad_vector_t CartpoleDynamicsPinocchio::systemFlowMap(
    ocs2::ad_scalar_t time,
    const ocs2::ad_vector_t& state,
    const ocs2::ad_vector_t& input,
    const ocs2::ad_vector_t& parameters) const {
  
  // OCS2 state: [theta, x, theta_dot, x_dot]
  // Pinocchio config q: [x, theta] (ordered by joint index in URDF)
  // Pinocchio velocity v: [x_dot, theta_dot]
  
  // Map OCS2 state to Pinocchio
  Eigen::Matrix<ocs2::ad_scalar_t, 2, 1> q;
  q(0) = state(1);      // x (slider position)
  q(1) = state(0);      // theta (hinge angle)

  Eigen::Matrix<ocs2::ad_scalar_t, 2, 1> v;
  v(0) = state(3);      // x_dot (slider velocity)
  v(1) = state(2);      // theta_dot (hinge velocity)

  // Force constraint: u applied to slider (x-direction)
  // tau = [u, 0] for [slider, hinge]
  Eigen::Matrix<ocs2::ad_scalar_t, 2, 1> tau;
  tau(0) = input(0);    // force on cart
  tau(1) = static_cast<ocs2::ad_scalar_t>(0.0);         // no torque on pole (free to rotate)

  // Use hardcoded physics equations (Pinocchio used for URDF loading only)
  // This allows proper AutoDiff support while using URDF-loaded parameters
  
  // Cartpole parameters from URDF
  const ocs2::ad_scalar_t m_cart = static_cast<ocs2::ad_scalar_t>(2.0);        // cart mass [kg]
  const ocs2::ad_scalar_t m_pole = static_cast<ocs2::ad_scalar_t>(0.2);        // pole mass [kg]
  const ocs2::ad_scalar_t l = static_cast<ocs2::ad_scalar_t>(1.0);             // pole length [m]
  const ocs2::ad_scalar_t g = static_cast<ocs2::ad_scalar_t>(9.81);            // gravity [m/s^2]
  
  const ocs2::ad_scalar_t cosTheta = cos(q(1));
  const ocs2::ad_scalar_t sinTheta = sin(q(1));

  // Dynamics from Euler-Lagrange equations
  // Inertia matrix I and RHS vector ordered as [x, theta]
  // (matching Pinocchio joint order: slider first, then hinge)
  Eigen::Matrix<ocs2::ad_scalar_t, 2, 2> I;
  I(0, 0) = m_cart + m_pole;                         // x equation: only mass terms
  I(0, 1) = m_pole * l * cosTheta / static_cast<ocs2::ad_scalar_t>(2.0);             // coupling term
  I(1, 0) = m_pole * l * cosTheta / static_cast<ocs2::ad_scalar_t>(2.0);             // coupling term (symmetric)
  I(1, 1) = m_pole * l * l / static_cast<ocs2::ad_scalar_t>(3.0);                    // theta equation: moment of inertia

  // Right-hand side vector (ordered as [x, theta])
  Eigen::Matrix<ocs2::ad_scalar_t, 2, 1> rhs;
  rhs(0) = tau(0) + m_pole * l / static_cast<ocs2::ad_scalar_t>(2.0) * v(1) * v(1) * sinTheta;  // x: input force + centrifugal
  rhs(1) = m_pole * l * g / static_cast<ocs2::ad_scalar_t>(2.0) * sinTheta;                      // theta: gravity torque

  // Solve for accelerations: I * a = rhs
  auto a = I.inverse() * rhs;
  // Now a = [a_x, a_theta]

  // State derivative ordered as OCS2 convention: [theta, x, theta_dot, x_dot]
  ocs2::ad_vector_t stateDerivative(4);
  stateDerivative(0) = v(1);           // d(theta)/dt = theta_dot
  stateDerivative(1) = v(0);           // d(x)/dt = x_dot
  stateDerivative(2) = a(1);           // d(theta_dot)/dt = theta_ddot (a(1) is a_theta)
  stateDerivative(3) = a(0);           // d(x_dot)/dt = x_ddot (a(0) is a_x)

  return stateDerivative;
}

}  // namespace ocs2_cartpole
