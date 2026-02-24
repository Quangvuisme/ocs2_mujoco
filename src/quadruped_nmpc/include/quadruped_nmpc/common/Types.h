#pragma once

#include <array>
#include <cstddef>

#include <ocs2_core/Types.h>

namespace quadruped_nmpc {

template<typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<ocs2::scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;

}  // namespace quadruped_nmpc
