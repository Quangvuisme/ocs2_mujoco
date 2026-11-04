#pragma once

#include <iostream>
#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace roadster_stunt {

/** Car and jet constants. Every number is an estimate, not a Tesla figure. */
struct RoadsterParameters {
  ocs2::scalar_t mass_ = 2000.0;          // [kg] incl. a full air tank
  ocs2::scalar_t rollInertia_ = 887.1;    // [kg m^2] solid 4.3 x 2.0 x 1.15 m box
  ocs2::scalar_t pitchInertia_ = 3302.1;  // [kg m^2]
  ocs2::scalar_t yawInertia_ = 3748.3;    // [kg m^2]
  ocs2::scalar_t gravity_ = 9.81;         // [m/s^2]
  ocs2::scalar_t jetY_ = 0.85;            // lateral offset of the lift and roof jets [m]
  ocs2::scalar_t liftMax_ = 15000.0;      // per lift jet [N]
  ocs2::scalar_t roofMax_ = 5000.0;       // per roof jet [N]
  ocs2::scalar_t sideMax_ = 3000.0;       // per side jet [N]
  ocs2::scalar_t impulseBudget_ = 180000.0;  // [N s] about 260 kg of air at a cold-gas Isp of 70 s
  ocs2::scalar_t rideHeight_ = 0.5328;    // body height at rest on the wheels [m]
  ocs2::scalar_t touchHeight_ = 0.64;     // body height when the relaxed tires touch the ground [m]
};

/** Kickflip scene and the shape of the stunt reference. */
struct StuntParameters {
  ocs2::scalar_t launchSpeed_ = 18.0;     // run-up speed [m/s]
  ocs2::scalar_t runup_ = 12.0;           // distance driven on the ground before the launch line [m]
  ocs2::scalar_t rowStart_ = 16.0;        // the parked row starts this far past the launch line [m]
  ocs2::scalar_t rowLength_ = 14.0;       // [m]
  ocs2::scalar_t rowHeight_ = 4.0;        // tallest vehicle in the row [m]
  ocs2::scalar_t rowMargin_ = 1.5;        // clearance zone starts/ends this far before/after the row [m]
  ocs2::scalar_t clearanceRadius_ = 1.2;  // half-diagonal of the car cross-section, rounded up [m]
  ocs2::scalar_t flightTime_ = 2.6;       // planned launch-to-touchdown time [s]
  ocs2::scalar_t apexMargin_ = 0.6;       // reference cruise height above the clearance height [m]
  ocs2::scalar_t rollStartTime_ = 0.55;   // the reference roll starts [s after launch]
  ocs2::scalar_t rollEndTime_ = 1.95;     // the reference roll ends [s after launch]
  ocs2::scalar_t flareHeight_ = 0.15;     // the plan ends this far above touchdown height ... [m]
  ocs2::scalar_t landingSinkRate_ = 0.4;  // ... sinking at this rate, which the NMPC keeps down to the ground [m/s]
};

inline RoadsterParameters loadRoadsterParameters(const std::string& filename, const std::string& fieldName = "roadster",
                                                 bool verbose = true) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  RoadsterParameters p;
  if (verbose) {
    std::cerr << "\n #### Roadster Parameters:";
    std::cerr << "\n #### =============================================================================\n";
  }
  ocs2::loadData::loadPtreeValue(pt, p.mass_, fieldName + ".mass", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rollInertia_, fieldName + ".rollInertia", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.pitchInertia_, fieldName + ".pitchInertia", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.yawInertia_, fieldName + ".yawInertia", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.gravity_, fieldName + ".gravity", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.jetY_, fieldName + ".jetY", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.liftMax_, fieldName + ".liftMax", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.roofMax_, fieldName + ".roofMax", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.sideMax_, fieldName + ".sideMax", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.impulseBudget_, fieldName + ".impulseBudget", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rideHeight_, fieldName + ".rideHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.touchHeight_, fieldName + ".touchHeight", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
  return p;
}

inline StuntParameters loadStuntParameters(const std::string& filename, const std::string& fieldName = "stunt",
                                           bool verbose = true) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  StuntParameters p;
  if (verbose) {
    std::cerr << "\n #### Stunt Parameters:";
    std::cerr << "\n #### =============================================================================\n";
  }
  ocs2::loadData::loadPtreeValue(pt, p.launchSpeed_, fieldName + ".launchSpeed", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.runup_, fieldName + ".runup", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rowStart_, fieldName + ".rowStart", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rowLength_, fieldName + ".rowLength", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rowHeight_, fieldName + ".rowHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rowMargin_, fieldName + ".rowMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.clearanceRadius_, fieldName + ".clearanceRadius", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.flightTime_, fieldName + ".flightTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.apexMargin_, fieldName + ".apexMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rollStartTime_, fieldName + ".rollStartTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.rollEndTime_, fieldName + ".rollEndTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.flareHeight_, fieldName + ".flareHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, p.landingSinkRate_, fieldName + ".landingSinkRate", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
  return p;
}

}  // namespace roadster_stunt
