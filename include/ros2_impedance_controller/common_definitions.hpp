// Copyright (c) 2026, qleonardolp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_IMPEDANCE_CONTROLLER__COMMON_DEFINITIONS_HPP_
#define ROS2_IMPEDANCE_CONTROLLER__COMMON_DEFINITIONS_HPP_

#include "Eigen/Dense"

namespace ros2_impedance_controller
{
const uint8_t kCartesianDim = 6;

using DiagonalMatrix6d = Eigen::DiagonalMatrix<double, kCartesianDim>;
using Vector6d = Eigen::Matrix<double, kCartesianDim, 1>;
using Matrix6d = Eigen::Matrix<double, kCartesianDim, kCartesianDim>;
}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__COMMON_DEFINITIONS_HPP_
