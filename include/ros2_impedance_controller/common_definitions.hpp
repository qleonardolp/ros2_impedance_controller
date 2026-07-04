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

/**
 * \brief Eigen-based Sliding Window
 *
 * Set a matrix with N vectors along the time window. Vectors are row-wise with time
 * dimension along the columns to leverage column-major (default) memory storage.
 */
class SlidingWindow
{
public:
  SlidingWindow(size_t window_size, size_t vector_size) : window_size_(window_size)
  {
    buffer_.resize(window_size, vector_size);
    buffer_.setZero();
  }

  int push(const Eigen::VectorXd entry)
  {
    if (entry.size() != buffer_.row(0).size())
    {
      return 1;
    }

    buffer_.bottomRows(window_size_ - 1) = buffer_.topRows(window_size_ - 1);  //
    buffer_.row(0) = entry;
    return 0;
  }

  Eigen::MatrixXd get_buffer() { return buffer_; }

private:
  Eigen::MatrixXd buffer_;
  size_t window_size_{1};
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__COMMON_DEFINITIONS_HPP_
