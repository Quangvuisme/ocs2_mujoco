#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace g1_walking {

/**
 * Joint posture reference for a commanded pelvis height.
 *
 * The MPC tracks a full joint posture besides the base pose. With a fixed default posture, a height
 * command is fought by the joint tracking cost. This IK keeps the feet flat on the footprint of the
 * default posture (same x/y under the pelvis) and moves the pelvis up / down by `heightOffset`:
 * a damped least-squares solve of the 6D pose of every foot frame relative to the pelvis.
 */
class LegPostureIk {
 public:
  /**
   * @param [in] pinocchioInterface : model with a free-flyer root and the actuated leg joints.
   * @param [in] defaultJoints : default posture (actuated joints, Pinocchio order).
   * @param [in] footFrames : one frame per foot, its 6D pose is kept (e.g. the ankle roll links).
   */
  LegPostureIk(const ocs2::PinocchioInterface& pinocchioInterface, ocs2::vector_t defaultJoints,
               std::vector<std::string> footFrames);

  /** Joint angles with the pelvis `heightOffset` [m] above (> 0) or below (< 0) the default posture. */
  const ocs2::vector_t& solve(double heightOffset);

 private:
  ocs2::PinocchioInterface pinocchioInterface_;
  ocs2::vector_t defaultJoints_;
  std::vector<size_t> footFrameIds_;
  // foot poses relative to the pelvis in the default posture
  std::vector<Eigen::Matrix3d> defaultFootRotations_;
  std::vector<Eigen::Vector3d> defaultFootPositions_;

  double cachedOffset_ = 0.0;
  ocs2::vector_t cachedJoints_;
};

}  // namespace g1_walking
