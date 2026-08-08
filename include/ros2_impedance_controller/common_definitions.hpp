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

#include <memory>
#include "Eigen/Dense"

namespace ros2_impedance_controller
{
const uint8_t kCartesianDim = 6;

using DiagonalMatrix6d = Eigen::DiagonalMatrix<double, kCartesianDim>;
using Vector6d = Eigen::Matrix<double, kCartesianDim, 1>;
using Matrix6d = Eigen::Matrix<double, kCartesianDim, kCartesianDim>;

const double kCmdAlpha{0.7585469929947761};  // LPF alpha for Nyquist frequency
const double kLPFAlpha{0.3858695450950376};  // LPF alpha for 1/10 sampling frequency

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
    // Roll rows
    for (size_t n = window_size_ - 1; n > 0; --n)
    {
      buffer_.row(n) = buffer_.row(n - 1);
    }
    buffer_.row(0) = entry;
    return 0;
  }

  Eigen::MatrixXd get_buffer() { return buffer_; }

private:
  Eigen::MatrixXd buffer_;
  size_t window_size_{1};
};

/**
 * \brief Impedance space planar identification (ISPI)
 *
 * Least square plane fitting from "Estimating Surface Normals in Noisy Point Cloud Data".
 * Ref.: https://dl.acm.org/doi/pdf/10.1145/777792.777840
 */
class ZSpaceIdentification
{
public:
  /**
   * @brief Constructor
   *
   * @param window_size Time window size. Must be at least 3.
   */
  explicit ZSpaceIdentification(size_t window_size) : zspace_window_(window_size)
  {
    if (window_size < 3)
    {
      window_size = 3;
      zspace_window_ = window_size;
    }
    zspace_points_ = std::make_shared<SlidingWindow>(window_size, 3);
    zspace_normal_last_ << 1.0, 0.0, 0.0;
    zspace_M_.setZero();
    zspace_is_steady_ = false;
    zspace_counter_ = 0;
  }

  /**
   * @brief Estimation update method
   *
   * Should be called in the main loop. It handle the time window downsample internally.
   */
  int update(
    const Vector6d deviation, const Vector6d twist, const Vector6d accel, const size_t axis)
  {
    // Check data integrity
    if (deviation.hasNaN() || twist.hasNaN() || accel.hasNaN()) return 1;

    zspace_new_ << deviation(axis), twist(axis), accel(axis);
    // Check steady state
    zspace_is_steady_ = zspace_new_.head<2>().norm() < 1e-3;  // accel is noisy so we neglect it
    if (zspace_is_steady_)
    {
      zspace_counter_ = 0;
      return 2;
    }

    zspace_points_->push(zspace_new_);
    zspace_counter_++;
    if (zspace_counter_ % zspace_window_ == 0)
    {
      zspace_mean_ = zspace_points_->get_buffer().colwise().mean();  // window mean
      for (size_t i = 0; i < zspace_window_; i++)
      {
        // Reusing `zspace_new_` as the centered point auxiliary variable
        zspace_new_.noalias() = zspace_points_->get_buffer().row(i);
        zspace_new_ -= zspace_mean_;
        zspace_M_ += zspace_new_ * zspace_new_.transpose();
      }
      zspace_M_ = zspace_M_ / static_cast<double>(zspace_window_);
      zspace_Meig_.compute(zspace_M_);
      // Eigenvector corresponding to the smallest eigenvalue:
      zspace_normal_ = zspace_Meig_.eigenvectors().col(0);
      // Neglect negative params (k,d,m):
      if (zspace_normal_(0) > 0 && zspace_normal_(1) > 0 && zspace_normal_(2) > 0)
      {
        zspace_normal_last_ = kLPFAlpha * zspace_normal_ + (1.0 - kLPFAlpha) * zspace_normal_last_;
      }
      zspace_M_.setZero();
      zspace_counter_ = 0;
      return 0;
    }
    return 3;
  }

  void reset_estimation()
  {
    zspace_M_.setZero();
    zspace_normal_last_ << 1.0, 0.0, 0.0;
    zspace_is_steady_ = false;
    zspace_counter_ = 0;
  }

  /**
   * @brief Return plane normal estimation in the impedance space
   */
  Eigen::Vector3d get_normal() { return zspace_normal_last_; }

  /**
   * @brief Return zspace points centroid (mean) for the last window
   */
  Eigen::Vector3d get_centroid() { return zspace_mean_; }

private:
  size_t zspace_window_;
  size_t zspace_counter_;
  std::shared_ptr<SlidingWindow> zspace_points_;
  Eigen::Vector3d zspace_new_;
  Eigen::Vector3d zspace_mean_;
  Eigen::Matrix3d zspace_M_;  // total least square fitting (hyper)plane
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> zspace_Meig_;  // M EigenSolver
  Eigen::Vector3d zspace_normal_last_;
  Eigen::Vector3d zspace_normal_;
  bool zspace_is_steady_;
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__COMMON_DEFINITIONS_HPP_
