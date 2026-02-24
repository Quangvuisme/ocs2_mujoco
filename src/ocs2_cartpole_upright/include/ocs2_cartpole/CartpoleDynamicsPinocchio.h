#pragma once

#include <memory>
#include <string>

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace ocs2_cartpole {

/**
 * CartPole System Dynamics using OCS2 PinocchioInterface + URDF
 * 
 * State vector: [theta, x, theta_dot, x_dot]
 *   theta: pole angle from vertical [rad]
 *   x: cart position [m]
 *   theta_dot: pole angular velocity [rad/s]
 *   x_dot: cart velocity [m/s]
 * 
 * Input vector: [u]
 *   u: applied force to cart [N]
 * 
 * Uses OCS2's PinocchioInterface for proper URDF handling and AutoDiff support
 */
class CartpoleDynamicsPinocchio : public ocs2::SystemDynamicsBaseAD {
 public:
  /**
   * Constructor
   * @param pinocchioInterface OCS2 PinocchioInterface object
   * @param library_folder Path to generate CppAD library
   * @param verbose Print debug information
   */
  CartpoleDynamicsPinocchio(const ocs2::PinocchioInterface& pinocchioInterface,
                            const std::string& library_folder, 
                            bool verbose = false);

  ~CartpoleDynamicsPinocchio() override = default;

  CartpoleDynamicsPinocchio(const CartpoleDynamicsPinocchio& rhs);

  CartpoleDynamicsPinocchio* clone() const override {
    return new CartpoleDynamicsPinocchio(*this);
  }

  /**
   * System dynamics function for automatic differentiation
   */
  ocs2::ad_vector_t systemFlowMap(
      ocs2::ad_scalar_t time,
      const ocs2::ad_vector_t& state,
      const ocs2::ad_vector_t& input,
      const ocs2::ad_vector_t& parameters) const override;

 private:
  std::shared_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
};

}  // namespace ocs2_cartpole
