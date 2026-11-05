#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_backflip_mujoco/BackflipGuide.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "g1_backflip_mujoco/BackflipReferenceManager.h"
#include "g1_backflip_mujoco/common/LoadVector.h"

namespace g1_backflip {

namespace {

/** Cubic Hermite interpolation with Catmull-Rom tangents (zero at the ends). */
template <typename Vector>
Vector hermite(const std::vector<double>& times, const std::vector<Vector>& values, double t) {
  const size_t n = times.size();
  if (t <= times.front()) {
    return values.front();
  }
  if (t >= times.back()) {
    return values.back();
  }
  size_t k = 0;
  while (k + 1 < n - 1 && t > times[k + 1]) {
    ++k;
  }
  auto tangent = [&](size_t i) -> Vector {
    if (i == 0 || i == n - 1) {
      return Vector(values[i] * 0.0);
    }
    return Vector((values[i + 1] - values[i - 1]) / (times[i + 1] - times[i - 1]));
  };
  const double h = times[k + 1] - times[k];
  const double s = (t - times[k]) / h;
  const double h00 = 2 * s * s * s - 3 * s * s + 1, h10 = s * s * s - 2 * s * s + s;
  const double h01 = -2 * s * s * s + 3 * s * s, h11 = s * s * s - s * s;
  return Vector(h00 * values[k] + h10 * h * tangent(k) + h01 * values[k + 1] + h11 * h * tangent(k + 1));
}

constexpr double kGravity = 9.81;
constexpr size_t kNumContacts = 2;

}  // namespace

BackflipGuide::BackflipGuide(const std::string& taskFile, const ocs2::PinocchioInterface& pinocchioInterface,
                             std::vector<std::string> contactFrames)
    : pinocchioInterface_(pinocchioInterface), contactFrames_(std::move(contactFrames)) {
  const auto& model = pinocchioInterface_.getModel();
  numJoints_ = model.nq - 6;
  mass_ = pinocchio::computeTotalMass(model);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  for (const auto& p : pt.get_child("postures")) {
    postures_.emplace_back(p.first, loadVectorString(pt, "postures." + p.first, numJoints_));
  }
  liftoffTime_ = pt.get<double>("guide.liftoffTime");
  touchdownTime_ = pt.get<double>("guide.touchdownTime");
  landingOffsetX_ = pt.get<double>("guide.landingOffsetX");
  for (const auto& k : pt.get_child("guide.keyframes")) {
    std::istringstream line(k.second.get_value<std::string>());
    Keyframe f;
    line >> f.time >> f.posture >> f.pitch;
    const bool inAir = f.time > liftoffTime_ + 1e-9 && f.time < touchdownTime_ - 1e-9;
    if (inAir && !(line >> f.x >> f.z)) {
      throw std::invalid_argument("[BackflipGuide] keyframe in the air needs 'time posture pitch x z': " + k.second.get_value<std::string>());
    }
    posture(f.posture);  // throws if unknown
    if (!keyframes_.empty() && f.time <= keyframes_.back().time) {
      throw std::invalid_argument("[BackflipGuide] keyframe times must increase");
    }
    keyframes_.push_back(f);
  }
  if (keyframes_.size() < 2) {
    throw std::invalid_argument("[BackflipGuide] at least two keyframes needed");
  }

  // pelvis positions: feet on the take-off footprint before lift-off, on the landing footprint after touchdown;
  // the robot starts with the pelvis at x = 0 (the pelvis x / z of the keyframes in the air are in this frame)
  const Eigen::Vector3d takeoffAnchor = Eigen::Vector3d::Zero();
  const Eigen::Vector3d landingAnchor(landingOffsetX_, 0.0, 0.0);
  for (const auto& f : keyframes_) {
    const ocs2::vector_t& q = posture(f.posture);
    if (f.time <= liftoffTime_ + 1e-9) {
      keyBase_.push_back(baseFromAnchor(q, f.pitch, takeoffAnchor));
    } else if (f.time >= touchdownTime_ - 1e-9) {
      keyBase_.push_back(baseFromAnchor(q, f.pitch, landingAnchor));
    } else {
      keyBase_.emplace_back(f.x, 0.0, f.z);
    }
  }
  const double shift = keyBase_.front().x();
  for (size_t k = 0; k < keyBase_.size(); ++k) {
    const bool inAir = keyframes_[k].time > liftoffTime_ + 1e-9 && keyframes_[k].time < touchdownTime_ - 1e-9;
    if (!inAir) {
      keyBase_[k].x() -= shift;
    }
  }
}

const ocs2::vector_t& BackflipGuide::posture(const std::string& name) const {
  for (const auto& p : postures_) {
    if (p.first == name) {
      return p.second;
    }
  }
  throw std::invalid_argument("[BackflipGuide] unknown posture '" + name + "'");
}

Eigen::Vector3d BackflipGuide::baseFromAnchor(const ocs2::vector_t& q, double pitch, const Eigen::Vector3d& anchor) const {
  const auto& model = pinocchioInterface_.getModel();
  pinocchio::Data data(model);
  ocs2::vector_t qPin = ocs2::vector_t::Zero(model.nq);
  qPin(3) = M_PI_2;  // flip_base yaw of a robot that faces world +x
  qPin(5) = pitch;   // flip_base roll = robot pitch
  qPin.tail(numJoints_) = q;
  pinocchio::forwardKinematics(model, data, qPin);
  pinocchio::updateFramePlacements(model, data);
  const Eigen::Vector3d foot = data.oMf[model.getFrameId(contactFrames_.front())].translation();
  return Eigen::Vector3d(anchor.x() - foot.x(), 0.0, anchor.z() - foot.z());
}

BackflipGuide::Sample BackflipGuide::sample(double t) const {
  std::vector<double> times;
  std::vector<ocs2::vector_t> qs;
  std::vector<Eigen::Vector3d> ps;
  std::vector<Eigen::Matrix<double, 1, 1>> pitches;
  for (size_t k = 0; k < keyframes_.size(); ++k) {
    times.push_back(keyframes_[k].time);
    qs.push_back(posture(keyframes_[k].posture));
    ps.push_back(keyBase_[k]);
    pitches.emplace_back(Eigen::Matrix<double, 1, 1>(keyframes_[k].pitch));
  }
  Sample s;
  s.q = hermite(times, qs, t);
  s.p = hermite(times, ps, t);
  s.pitch = hermite(times, pitches, t)(0);
  return s;
}

ocs2::vector_t BackflipGuide::generalizedCoordinates(double t) const {
  const Sample s = sample(t);
  ocs2::vector_t q(6 + numJoints_);
  q << s.p, M_PI_2, 0.0, s.pitch, s.q;
  return q;
}

ocs2::ModeSchedule BackflipGuide::modeSchedule(double timeOffset) const {
  return backflipModeSchedule(timeOffset + liftoffTime_, timeOffset + touchdownTime_);
}

ocs2::vector_t BackflipGuide::state(double t) const {
  constexpr double kEps = 1e-3;
  ocs2::vector_t x(2 * (6 + numJoints_));
  x << generalizedCoordinates(t), (generalizedCoordinates(t + kEps) - generalizedCoordinates(t - kEps)) / (2 * kEps);
  return x;
}

ocs2::vector_t BackflipGuide::initialState() const {
  ocs2::vector_t x = state(0.0);
  x.tail(6 + numJoints_).setZero();
  return x;
}

ocs2::vector_t BackflipGuide::input(double t) const {
  ocs2::vector_t u = ocs2::vector_t::Zero(6 * kNumContacts + numJoints_);
  // joint accelerations
  constexpr double kEps = 2e-3;
  const ocs2::vector_t q0 = sample(t - kEps).q, q1 = sample(t).q, q2 = sample(t + kEps).q;
  u.tail(numJoints_) = (q2 - 2.0 * q1 + q0) / (kEps * kEps);
  // contact forces: m (a_com + g) shared by the feet, a_com from the guide's COM trajectory
  const bool stance = modeSchedule().modeAtTime(t) == ocs2::humanoid::ModeNumber::STANCE;
  if (stance) {
    const auto& model = pinocchioInterface_.getModel();
    pinocchio::Data data(model);
    constexpr double kDt = 0.01;
    const Eigen::Vector3d c0 = pinocchio::centerOfMass(model, data, generalizedCoordinates(t - kDt));
    const Eigen::Vector3d c1 = pinocchio::centerOfMass(model, data, generalizedCoordinates(t));
    const Eigen::Vector3d c2 = pinocchio::centerOfMass(model, data, generalizedCoordinates(t + kDt));
    Eigen::Vector3d f = mass_ * ((c2 - 2.0 * c1 + c0) / (kDt * kDt) + Eigen::Vector3d(0.0, 0.0, kGravity)) / kNumContacts;
    f.z() = std::max(f.z(), 0.0);
    for (size_t i = 0; i < kNumContacts; ++i) {
      u.segment<3>(6 * i) = f;
    }
  }
  return u;
}

ocs2::TargetTrajectories BackflipGuide::targetTrajectories(double dt, double timeOffset) const {
  ocs2::TargetTrajectories targets;
  const int n = static_cast<int>(std::ceil(endTime() / dt));
  for (int k = 0; k <= n; ++k) {
    const double t = std::min(k * dt, endTime());
    ocs2::vector_t x = state(t);
    if (k == 0 || k == n) {
      x.tail(6 + numJoints_).setZero();
    }
    targets.timeTrajectory.push_back(timeOffset + t);
    targets.stateTrajectory.push_back(std::move(x));
    targets.inputTrajectory.push_back(input(t));
  }
  return targets;
}

}  // namespace g1_backflip
