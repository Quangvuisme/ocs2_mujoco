#include "anymal_perceptive_mujoco/foot_planner/SwingTrajectoryPlanner.h"

#include <cmath>
#include <iostream>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Lookup.h>

#include "anymal_perceptive_mujoco/gait/MotionPhaseDefinition.h"

namespace anymal_perceptive {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) : config_(config), numFeet_(numFeet) {}

ocs2::scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, ocs2::scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].velocity(time);
}

ocs2::scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, ocs2::scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].position(time);
}

void SwingTrajectoryPlanner::update(const ocs2::ModeSchedule& modeSchedule, ocs2::scalar_t terrainHeight) {
  const ocs2::scalar_array_t heights(modeSchedule.modeSequence.size(), terrainHeight);
  const ocs2::scalar_array_t noObstacle(modeSchedule.modeSequence.size(), std::numeric_limits<ocs2::scalar_t>::quiet_NaN());
  feet_array_t<ocs2::scalar_array_t> liftOff, touchDown, obstacle;
  liftOff.fill(heights);
  touchDown.fill(heights);
  obstacle.fill(noObstacle);
  update(modeSchedule, liftOff, touchDown, obstacle);
}

void SwingTrajectoryPlanner::update(const ocs2::ModeSchedule& modeSchedule,
                                    const feet_array_t<ocs2::scalar_array_t>& liftOffHeightSequence,
                                    const feet_array_t<ocs2::scalar_array_t>& touchDownHeightSequence,
                                    const feet_array_t<ocs2::scalar_array_t>& obstacleHeightSequence) {
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    feetHeightTrajectories_[j].clear();
    feetHeightTrajectories_[j].reserve(modeSequence.size());
    for (size_t p = 0; p < modeSequence.size(); ++p) {
      if (!eesContactFlagStocks[j][p]) {  // swing leg
        const int swingStartIndex = startTimesIndices[j][p];
        const int swingFinalIndex = finalTimesIndices[j][p];
        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

        const ocs2::scalar_t swingStartTime = eventTimes[swingStartIndex];
        const ocs2::scalar_t swingFinalTime = eventTimes[swingFinalIndex];
        const ocs2::scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

        const ocs2::scalar_t liftOffHeight = liftOffHeightSequence[j][p];
        const ocs2::scalar_t touchDownHeight = touchDownHeightSequence[j][p];
        ocs2::scalar_t clearBase = std::max(liftOffHeight, touchDownHeight);
        const ocs2::scalar_t obstacle = obstacleHeightSequence[j][p];
        if (std::isfinite(obstacle)) {
          clearBase = std::max(clearBase, obstacle);
        }

        const CubicSpline::Node liftOff{swingStartTime, liftOffHeight, scaling * config_.liftOffVelocity};
        const CubicSpline::Node touchDown{swingFinalTime, touchDownHeight, scaling * config_.touchDownVelocity};
        const ocs2::scalar_t midHeight = clearBase + scaling * config_.swingHeight;
        feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);
      } else {  // stance leg
        // Note: setting the time here arbitrarily to 0.0 -> 1.0 makes the assert in CubicSpline fail
        const ocs2::scalar_t h = liftOffHeightSequence[j][p];
        const CubicSpline::Node liftOff{0.0, h, 0.0};
        const CubicSpline::Node touchDown{1.0, h, 0.0};
        feetHeightTrajectories_[j].emplace_back(liftOff, h, touchDown);
      }
    }
    feetHeightTrajectoriesEvents_[j] = eventTimes;
  }
}

std::pair<std::vector<int>, std::vector<int>> SwingTrajectoryPlanner::updateFootSchedule(
    const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();
  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);
  for (size_t i = 0; i < numPhases; i++) {
    if (!contactFlagStock[i]) {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

feet_array_t<std::vector<bool>> SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();
  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));
  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < numFeet_; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();
  if (contactFlagStock[index]) {
    return {0, 0};
  }
  int startTimesIndex = -1;
  for (int ip = static_cast<int>(index) - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }
  int finalTimesIndex = static_cast<int>(numPhases) - 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = static_cast<int>(ip) - 1;
      break;
    }
  }
  return {startTimesIndex, finalTimesIndex};
}

void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock) {
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0) {
    throw std::runtime_error("[SwingTrajectoryPlanner] The take-off time of the first swing of leg " + std::to_string(leg) +
                             " (phase " + std::to_string(index) + ") is not defined.");
  }
  if (finalIndex >= static_cast<int>(numSubsystems) - 1) {
    throw std::runtime_error("[SwingTrajectoryPlanner] The touch-down time of the last swing of leg " + std::to_string(leg) +
                             " (phase " + std::to_string(index) + ") is not defined.");
  }
}

ocs2::scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(ocs2::scalar_t startTime, ocs2::scalar_t finalTime,
                                                              ocs2::scalar_t swingTimeScale) {
  return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}

SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName, const std::string& fieldName,
                                                           bool verbose) {
  boost::property_tree::ptree pt;
  read_info(fileName, pt);
  SwingTrajectoryPlanner::Config config;
  const std::string prefix = fieldName + ".";
  ocs2::loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);
  return config;
}

}  // namespace anymal_perceptive
