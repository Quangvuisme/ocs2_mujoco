#pragma once

#include <string>
#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace g1_backflip {

/**
 * Hand-designed guide of the backflip (config/task.info, blocks "postures" and "guide"), for the whole-body state
 * [q, v] = [base position, flip_base ZYX Euler, joints, base linear velocity, Euler rates, joint velocities] and
 * input [left foot wrench, right foot wrench, joint accelerations].
 *
 * Keyframes give the time, a named joint posture, the robot pitch (the flip_base roll: 0 upright, -2 pi after the
 * flip) and, in the air, the pelvis x / z. On the ground the pelvis position follows from the posture and the pitch
 * so that the foot contact frames stay on the take-off (or landing) footprint: the guide is kinematically consistent
 * in stance. Postures, pitch and pelvis position are cubic Hermite splines (Catmull-Rom tangents); velocities and
 * joint accelerations are their derivatives, the foot forces carry the guide's COM acceleration plus gravity.
 *
 * Times are relative to the start of the manoeuvre (t = 0, standing still).
 */
class BackflipGuide {
 public:
  struct Keyframe {
    double time = 0.0;
    std::string posture;
    double pitch = 0.0;
    double x = 0.0;  // pelvis x / z, used in the air only
    double z = 0.0;
  };

  /** @param pinocchioInterface : the MPC model (free-flyer = translation + ZYX Euler, MPC joints, contact frames). */
  BackflipGuide(const std::string& taskFile, const ocs2::PinocchioInterface& pinocchioInterface, std::vector<std::string> contactFrames);

  double liftoffTime() const { return liftoffTime_; }
  double touchdownTime() const { return touchdownTime_; }
  double endTime() const { return keyframes_.back().time; }
  ocs2::ModeSchedule modeSchedule(double timeOffset = 0.0) const;

  /** Target trajectories sampled every dt over [0, endTime], shifted by timeOffset. */
  ocs2::TargetTrajectories targetTrajectories(double dt, double timeOffset = 0.0) const;

  /** Guide state at time t (velocities from the splines); zero velocity at the first / last keyframe. */
  ocs2::vector_t state(double t) const;
  ocs2::vector_t input(double t) const;
  ocs2::vector_t initialState() const;

  const ocs2::vector_t& posture(const std::string& name) const;

 private:
  struct Sample {
    ocs2::vector_t q;   // joints
    double pitch;
    Eigen::Vector3d p;  // pelvis (= flip_base origin) position
  };
  Sample sample(double t) const;
  ocs2::vector_t generalizedCoordinates(double t) const;
  /** Base position that puts the left foot contact frame of the posture at the anchor (x, z). */
  Eigen::Vector3d baseFromAnchor(const ocs2::vector_t& q, double pitch, const Eigen::Vector3d& anchor) const;

  ocs2::PinocchioInterface pinocchioInterface_;
  std::vector<std::string> contactFrames_;
  size_t numJoints_ = 0;
  double mass_ = 0.0;

  std::vector<std::pair<std::string, ocs2::vector_t>> postures_;
  std::vector<Keyframe> keyframes_;
  std::vector<Eigen::Vector3d> keyBase_;  // pelvis position of every keyframe
  double liftoffTime_ = 0.0, touchdownTime_ = 0.0;
  double landingOffsetX_ = 0.0;
};

}  // namespace g1_backflip
