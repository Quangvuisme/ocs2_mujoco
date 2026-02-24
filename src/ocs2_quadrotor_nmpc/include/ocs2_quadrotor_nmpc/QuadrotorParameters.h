#pragma once

#include <iostream>
#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2_cartpole_quadrotor {

struct QuadrotorParameters {
  ocs2::scalar_t quadrotorMass_ = 0.546;  // [kg]
  ocs2::scalar_t Thzz_ = 3e-4;            // Inertia around z-axis
  ocs2::scalar_t Thxxyy_ = 2.32e-3;       // Inertia around x/y-axis
  ocs2::scalar_t gravity_ = 9.81;         // [m/s^2]
};

inline QuadrotorParameters loadSettings(const std::string& filename, 
                                        const std::string& fieldName = "QuadrotorParameters",
                                        bool verbose = true) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  QuadrotorParameters settings;

  if (verbose) {
    std::cerr << "\n #### Quadrotor Parameters:";
    std::cerr << "\n #### =============================================================================\n";
  }

  ocs2::loadData::loadPtreeValue(pt, settings.quadrotorMass_, fieldName + ".quadrotorMass", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.Thzz_, fieldName + ".Thzz", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.Thxxyy_, fieldName + ".Thxxyy", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.gravity_, fieldName + ".gravity", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace ocs2_cartpole_quadrotor
