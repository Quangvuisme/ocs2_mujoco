#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_perceptive_mujoco/reference_manager/PerceptiveReferenceManager.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Lookup.h>

namespace g1_perceptive {

namespace {
constexpr double kGravity = 9.81;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

constexpr size_t kNumFeet = 2;  // foot f: toe contact f, heel contact f + kNumFeet

Eigen::Matrix2d yawRotation(double yaw) {
  Eigen::Matrix2d R;
  R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  return R;
}
}  // namespace

PerceptiveReferenceManager::PerceptiveReferenceManager(ocs2::CentroidalModelInfo info,
                                                       std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                       std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                       const ocs2::PinocchioInterface& pinocchioInterface,
                                                       const ocs2::vector_t& nominalState, PerceptionSettings settings,
                                                       std::shared_ptr<TerrainMapBuffer> terrainBuffer)
    : SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      info_(std::move(info)),
      pinocchioInterface_(pinocchioInterface),
      settings_(std::move(settings)),
      terrainBuffer_(std::move(terrainBuffer)),
      flatTerrain_(TerrainMap::flat(0.0)),
      terrain_(flatTerrain_),
      footholdPlanner_(settings_.foothold) {
  // foot positions of the nominal stance in the base yaw frame
  const auto feet = computeFeetPositions(nominalState);
  const Eigen::Vector3d basePosition = ocs2::centroidal_model::getBasePose(nominalState, info_).head<3>();
  const double yaw = nominalState(9);
  for (size_t leg = 0; leg < feet.size(); ++leg) {
    nominalFootInBase_[leg] = yawRotation(yaw).transpose() * (feet[leg] - basePosition).head<2>();
  }
  // 6D IK of the ankle roll links (sole frames) for the posture reference
  nominalBaseHeight_ = nominalState(8);
  postureIk_ = std::make_unique<LegPostureIk>(pinocchioInterface_, ocs2::centroidal_model::getJointAngles(nominalState, info_),
                                              std::vector<std::string>{"left_ankle_roll_link", "right_ankle_roll_link"});
  currentStanceFinalTime_.fill(0.0);
  for (auto& p : lastStancePosition_) {
    p.setZero();
  }
}

void PerceptiveReferenceManager::setCommandTrajectories(ocs2::TargetTrajectories command) {
  std::lock_guard<std::mutex> lock(commandMutex_);
  command_ = std::move(command);
}

feet_array_t<vector3_t> PerceptiveReferenceManager::computeFeetPositions(const ocs2::vector_t& state) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const ocs2::vector_t q = ocs2::centroidal_model::getGeneralizedCoordinates(state, info_);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  feet_array_t<vector3_t> feet;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    feet[i] = data.oMf[info_.endEffectorFrameIndices[i]].translation();
  }
  return feet;
}

Eigen::Vector2d PerceptiveReferenceManager::nominalFootXY(size_t leg, const ocs2::vector_t& state) const {
  return Eigen::Vector2d(state(6), state(7)) + yawRotation(state(9)) * nominalFootInBase_[leg];
}

const Foothold* PerceptiveReferenceManager::getFoothold(size_t leg, ocs2::scalar_t time) const {
  if (footholds_[leg].empty()) {
    return nullptr;
  }
  const size_t index = ocs2::lookup::findIndexInTimeArray(eventTimes_, time);
  if (index >= footholds_[leg].size() || !contactFlags_[leg][index]) {
    return nullptr;
  }
  return &footholds_[leg][index];
}

bool PerceptiveReferenceManager::isFootPlacementActive(size_t leg, ocs2::scalar_t time) const {
  if (time <= currentStanceFinalTime_[leg]) {
    return false;
  }
  const Foothold* f = getFoothold(leg, time);
  return f != nullptr && f->constrained;
}

PerceptiveReferenceManager::Plan PerceptiveReferenceManager::getPlan() const {
  std::lock_guard<std::mutex> lock(planMutex_);
  return plan_;
}

void PerceptiveReferenceManager::modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                                                  const ocs2::vector_t& initState,
                                                  ocs2::TargetTrajectories& targetTrajectories,
                                                  ocs2::ModeSchedule& modeSchedule) {
  modeSchedule = updateModeSchedule(initTime, finalTime);

  auto latest = terrainBuffer_ ? terrainBuffer_->get() : nullptr;
  terrain_ = latest ? latest : flatTerrain_;

  targetTrajectories = liftCommandOntoTerrain(initTime, finalTime, initState);
  planFootholds(initTime, finalTime, initState, targetTrajectories, modeSchedule);
  planSwingTrajectories(modeSchedule);

  // visualisation copy
  Plan plan;
  plan.time = initTime;
  plan.terrain = terrain_;
  for (const auto& x : targetTrajectories.stateTrajectory) {
    plan.baseTrajectory.emplace_back(x(6), x(7), x(8));
  }
  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    for (size_t p = 0; p < footholds_[leg].size(); ++p) {
      const bool startsStance = contactFlags_[leg][p] && (p == 0 || !contactFlags_[leg][p - 1]);
      const double startTime = p == 0 ? -1e9 : eventTimes_[p - 1];
      if (startsStance && startTime >= initTime && startTime <= finalTime) {
        plan.footholds.emplace_back(leg, footholds_[leg][p]);
      }
    }
  }
  std::lock_guard<std::mutex> lock(planMutex_);
  plan_ = std::move(plan);
}

ocs2::TargetTrajectories PerceptiveReferenceManager::liftCommandOntoTerrain(ocs2::scalar_t initTime,
                                                                            ocs2::scalar_t finalTime,
                                                                            const ocs2::vector_t& initState) {
  ocs2::TargetTrajectories command;
  {
    std::lock_guard<std::mutex> lock(commandMutex_);
    command = command_;
  }
  if (command.empty()) {  // no command yet: hold the current pose at the current height above ground
    ocs2::vector_t x = initState;
    x.head(6).setZero();
    x(8) = initState(8) - terrain_->smoothHeightAt(Eigen::Vector2d(initState(6), initState(7)));
    command = ocs2::TargetTrajectories({initTime}, {x}, {ocs2::vector_t::Zero(info_.inputDim)});
  }

  const int numNodes = std::max(2, settings_.numTargetNodes);
  const double dt = (finalTime - initTime) / (numNodes - 1);
  ocs2::TargetTrajectories target;
  target.timeTrajectory.reserve(numNodes);
  for (int k = 0; k < numNodes; ++k) {
    const double t = initTime + k * dt;
    ocs2::vector_t x = command.getDesiredState(t);
    const ocs2::vector_t u = command.getDesiredInput(t);

    // least-squares plane z = a + bx * dx + by * dy through the terrain under the nominal footholds
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
      const double h = terrain_->smoothHeightAt(nominalFootXY(leg, x));
      const Eigen::Vector3d phi(1.0, nominalFootInBase_[leg].x(), nominalFootInBase_[leg].y());
      M += phi * phi.transpose();
      r += phi * h;
    }
    const Eigen::Vector3d plane = M.ldlt().solve(r);
    const double heightAboveTerrain = x(8);
    x(8) = plane(0) + heightAboveTerrain;
    const double slopePitch = -std::atan(plane(1));  // > 0: terrain goes down ahead (nose down)
    const double pitchGain = slopePitch > 0.0 ? settings_.basePitchGainDown : settings_.basePitchGain;
    x(10) = std::clamp(slopePitch * pitchGain, -settings_.maxBasePitch, settings_.maxBasePitch);
    x(11) = std::clamp(std::atan(plane(2)) * settings_.baseRollGain, -settings_.maxBaseRoll, settings_.maxBaseRoll);

    // posture reference: every sole flat on the terrain under it, pelvis at the lifted height.
    // Foot f moves by (terrain under it - pelvis height) - (0 - nominal pelvis height) relative to the pelvis.
    if (settings_.jointReferenceFromIk) {
      std::vector<double> footOffsets(kNumFeet);
      for (size_t f = 0; f < kNumFeet; ++f) {
        const Eigen::Vector2d centre = 0.5 * (nominalFootXY(f, x) + nominalFootXY(f + kNumFeet, x));
        footOffsets[f] = terrain_->smoothHeightAt(centre) - x(8) + nominalBaseHeight_;
      }
      ocs2::centroidal_model::getJointAngles(x, info_) = postureIk_->solveFeet(footOffsets);
    }

    target.timeTrajectory.push_back(t);
    target.stateTrajectory.push_back(std::move(x));
    target.inputTrajectory.push_back(u);
  }
  // vertical CoM velocity consistent with the lifted height profile
  for (int k = 0; k < numNodes; ++k) {
    const int k0 = std::max(0, k - 1);
    const int k1 = std::min(numNodes - 1, k + 1);
    target.stateTrajectory[k](2) =
        (target.stateTrajectory[k1](8) - target.stateTrajectory[k0](8)) / ((k1 - k0) * dt);
  }
  return target;
}

void PerceptiveReferenceManager::planFootholds(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                                               const ocs2::vector_t& initState,
                                               const ocs2::TargetTrajectories& targetTrajectories,
                                               const ocs2::ModeSchedule& modeSchedule) {
  const auto& modeSequence = modeSchedule.modeSequence;
  eventTimes_ = modeSchedule.eventTimes;
  contactFlags_ = getSwingTrajectoryPlanner()->extractContactFlags(modeSequence);
  const size_t numPhases = modeSequence.size();

  footholdPlanner_.beginIteration();
  const auto measuredFeet = computeFeetPositions(initState);
  const double ffh = settings_.footFrameHeight;

  // Raibert-style feedback on the velocity error
  const Eigen::Vector2d baseXY(initState(6), initState(7));
  const double heightAboveTerrain = std::clamp(initState(8) - terrain_->smoothHeightAt(baseXY), 0.3, 0.8);
  const ocs2::vector_t desiredNow = targetTrajectories.getDesiredState(initTime);
  const Eigen::Vector2d velocityError = initState.head<2>() - desiredNow.head<2>();
  Eigen::Vector2d feedback = settings_.foothold.feedbackGain * std::sqrt(heightAboveTerrain / kGravity) * velocityError;
  // clamp in the heading frame
  const Eigen::Matrix2d Ryaw = yawRotation(initState(9));
  Eigen::Vector2d feedbackLocal = Ryaw.transpose() * feedback;
  feedbackLocal.x() = std::clamp(feedbackLocal.x(), -settings_.foothold.maxFeedbackX, settings_.foothold.maxFeedbackX);
  feedbackLocal.y() = std::clamp(feedbackLocal.y(), -settings_.foothold.maxFeedbackY, settings_.foothold.maxFeedbackY);
  feedback = Ryaw * feedbackLocal;

  for (size_t foot = 0; foot < kNumFeet; ++foot) {
    const size_t toe = foot;
    const size_t heel = foot + kNumFeet;
    const auto& contact = contactFlags_[toe];  // toe and heel of a foot share the contact schedule
    for (const size_t c : {toe, heel}) {
      footholds_[c].assign(numPhases, Foothold());
      currentStanceFinalTime_[c] = initTime;  // swinging now: every later stance is constrained
    }
    const feet_array_t<Eigen::Vector3d> measuredContact = [&] {
      feet_array_t<Eigen::Vector3d> m;
      for (size_t c = 0; c < m.size(); ++c) {
        m[c] = measuredFeet[c] - ffh * Eigen::Vector3d::UnitZ();
      }
      return m;
    }();

    size_t i = 0;
    while (i < numPhases) {
      if (!contact[i]) {
        ++i;
        continue;
      }
      size_t j = i;
      while (j + 1 < numPhases && contact[j + 1]) {
        ++j;
      }
      const double startTime = i > 0 ? eventTimes_[i - 1] : -std::numeric_limits<double>::infinity();
      const double endTime = j + 1 < numPhases ? eventTimes_[j] : std::numeric_limits<double>::infinity();

      Foothold toeFoothold, heelFoothold;
      if (startTime <= initTime && initTime <= endTime) {
        // current stance: the foot stays where it is
        for (const size_t c : {toe, heel}) {
          lastStancePosition_[c] = measuredContact[c];
          hasLastStancePosition_[c] = true;
          currentStanceFinalTime_[c] = endTime;
        }
        toeFoothold = FootholdPlanner::fixed(measuredContact[toe], *terrain_);
        heelFoothold = FootholdPlanner::fixed(measuredContact[heel], *terrain_);
      } else if (endTime < initTime) {
        // past stance: where the foot lifted off
        toeFoothold = FootholdPlanner::fixed(hasLastStancePosition_[toe] ? lastStancePosition_[toe] : measuredContact[toe], *terrain_);
        heelFoothold =
            FootholdPlanner::fixed(hasLastStancePosition_[heel] ? lastStancePosition_[heel] : measuredContact[heel], *terrain_);
      } else {
        // future stance: Raibert foothold of the sole centre at mid-stance, toe and heel both on steppable cells
        const double stanceEnd = std::isfinite(endTime) ? endTime : startTime + 0.3;
        const double midTime = std::clamp(0.5 * (startTime + stanceEnd), initTime, finalTime);
        const ocs2::vector_t desired = targetTrajectories.getDesiredState(midTime);
        const Eigen::Vector2d toeXY = nominalFootXY(toe, desired) + feedback;
        const Eigen::Vector2d heelXY = nominalFootXY(heel, desired) + feedback;
        const Eigen::Vector2d centre = 0.5 * (toeXY + heelXY);
        const Eigen::Vector2d heading(std::cos(desired(9)), std::sin(desired(9)));
        const auto selected =
            footholdPlanner_.selectFoot(foot, startTime, centre, {toeXY - centre, heelXY - centre}, *terrain_, heading);
        toeFoothold = selected[0];
        heelFoothold = selected[1];
      }
      for (size_t k = i; k <= j; ++k) {
        footholds_[toe][k] = toeFoothold;
        footholds_[heel][k] = heelFoothold;
      }
      i = j + 1;
    }
  }
}

void PerceptiveReferenceManager::planSwingTrajectories(const ocs2::ModeSchedule& modeSchedule) {
  const size_t numPhases = modeSchedule.modeSequence.size();
  const double ffh = settings_.footFrameHeight;
  feet_array_t<ocs2::scalar_array_t> liftOff, touchDown, obstacle;
  for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
    const auto& contact = contactFlags_[leg];
    liftOff[leg].assign(numPhases, 0.0);
    touchDown[leg].assign(numPhases, 0.0);
    obstacle[leg].assign(numPhases, kNaN);
    for (size_t p = 0; p < numPhases; ++p) {
      if (contact[p]) {
        liftOff[leg][p] = touchDown[leg][p] = footholds_[leg][p].position.z() + ffh;
        continue;
      }
      int previous = static_cast<int>(p) - 1;
      while (previous >= 0 && !contact[previous]) {
        --previous;
      }
      size_t next = p + 1;
      while (next < numPhases && !contact[next]) {
        ++next;
      }
      const Eigen::Vector3d start = previous >= 0 ? footholds_[leg][previous].position : lastStancePosition_[leg];
      const Eigen::Vector3d end = next < numPhases ? footholds_[leg][next].position : start;
      liftOff[leg][p] = start.z() + ffh;
      touchDown[leg][p] = end.z() + ffh;
      obstacle[leg][p] = terrain_->maxInflatedHeightOnSegment(start.head<2>(), end.head<2>()) + ffh;
    }
  }
  getSwingTrajectoryPlanner()->update(modeSchedule, liftOff, touchDown, obstacle);
}

PerceptionSettings loadPerceptionSettings(const std::string& file, const std::string& prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  PerceptionSettings s;
  const std::string p = prefix + ".";
  ocs2::loadData::loadPtreeValue(pt, s.numTargetNodes, p + "numTargetNodes", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.basePitchGain, p + "basePitchGain", verbose);
  s.basePitchGainDown = s.basePitchGain;
  ocs2::loadData::loadPtreeValue(pt, s.basePitchGainDown, p + "basePitchGainDown", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.baseRollGain, p + "baseRollGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.maxBasePitch, p + "maxBasePitch", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.maxBaseRoll, p + "maxBaseRoll", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.jointReferenceFromIk, p + "jointReferenceFromIk", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.footFrameHeight, p + "footFrameHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.swingClearance, p + "swingClearance", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.swingConstraintLiftOffMargin, p + "swingConstraintLiftOffMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.swingConstraintTouchDownMargin, p + "swingConstraintTouchDownMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.enableFootPlacement, p + "enableFootPlacement", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.enableSwingClearance, p + "enableSwingClearance", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.enableShankClearance, p + "enableShankClearance", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.shankRadius, p + "shankRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.ankleClearance, p + "ankleClearance", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.shankClearanceMu, p + "shankClearanceMu", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.shankClearanceDelta, p + "shankClearanceDelta", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.footPlacementMu, p + "footPlacementMu", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.footPlacementDelta, p + "footPlacementDelta", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.swingClearanceMu, p + "swingClearanceMu", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.swingClearanceDelta, p + "swingClearanceDelta", verbose);
  s.foothold = loadFootholdPlannerConfig(file, prefix + ".foothold", verbose);
  return s;
}

}  // namespace g1_perceptive
