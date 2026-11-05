#pragma once

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/Types.h>

namespace g1_backflip {

/** Reads a whitespace-separated list of numbers stored as one (quoted) value of an .info file. */
inline ocs2::vector_t loadVectorString(const boost::property_tree::ptree& pt, const std::string& key, size_t expectedSize = 0) {
  const std::string text = pt.get<std::string>(key);
  std::istringstream stream(text);
  std::vector<double> values;
  double v;
  while (stream >> v) {
    values.push_back(v);
  }
  if (!stream.eof()) {
    throw std::invalid_argument("[loadVectorString] '" + key + "' is not a list of numbers: " + text);
  }
  if (expectedSize > 0 && values.size() != expectedSize) {
    throw std::invalid_argument("[loadVectorString] '" + key + "' has " + std::to_string(values.size()) + " values, expected " +
                                std::to_string(expectedSize));
  }
  return Eigen::Map<const ocs2::vector_t>(values.data(), values.size());
}

}  // namespace g1_backflip
