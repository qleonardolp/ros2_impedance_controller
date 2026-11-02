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

#include "ros2_impedance_controller/impedance_estimator.hpp"

namespace ros2_impedance_controller
{
ImpedanceEstimator::ImpedanceEstimator(uint window_size, uint axis, double delta_t)
: window_(window_size), axis_(axis), delta_t_(delta_t)
{
  if (window_size < kParamsDim) window_ = kParamsDim;

  double svf_wn_ = 100;
  double svf_zeta_ = 0.707;

  A_svf_ << 0, 1, -svf_wn_ * svf_wn_, -2 * svf_zeta_ * svf_wn_;
  B_svf_ << 0, svf_wn_ * svf_wn_;

  e_state_.setZero();
  fint_state_.setZero();

  // Concurrent Learning
  H_.setZero();
  zspace_stack_ = Eigen::MatrixXd::Zero(window_, kParamsDim);
  fint_stack_ = Eigen::VectorXd::Zero(window_);

  lowest_lambda_ = 0.0;
  num_stored_ = 0;
  subst_idx_ = 0;

  fsm_current_state_ = State::NULLSPACE;
}

int ImpedanceEstimator::update_state(
  const Vector6d deviation, const Vector6d twist, const Vector6d accel, const Vector6d wrench)
{
  // Check data integrity
  if (deviation.hasNaN() || twist.hasNaN() || accel.hasNaN() || wrench.hasNaN()) return 1;

  uint_fast8_t bitmask = ((abs(deviation(axis_)) < kDeviationThreshold) << 3) |
                         ((abs(twist(axis_)) < kTwistDevThreshold) << 2) |
                         ((abs(accel(axis_)) < kAccelDevThreshold) << 1) |
                         (abs(wrench(axis_)) < kIntEffortThreshold);
  fsm_current_state_ = StateLUT[bitmask];

  de_state_ = A_svf_ * e_state_ + B_svf_ * accel(axis_);
  dfint_state_ = A_svf_ * fint_state_ + B_svf_ * wrench(axis_);

  // Euler integration
  e_state_ += de_state_ * delta_t_;
  fint_state_ += dfint_state_ * delta_t_;

  zspace_new_point_ << deviation(axis_), twist(axis_), e_state_(0);
  fint_new_ = fint_state_(0);
  return 0;
}

int ImpedanceEstimator::update()
{
  zTz_new_ = zspace_new_point_ * zspace_new_point_.transpose();

  // Initial buffer phase
  if (num_stored_ < window_)
  {
    zspace_stack_.row(num_stored_) = zspace_new_point_.transpose();
    fint_stack_(num_stored_) = fint_new_;
    H_ += zTz_new_;
    num_stored_++;
    return 0;
  }

  // O(N) evaluation
  subst_idx_ = -1;
  solver_H_.compute(H_);
  lowest_lambda_ = solver_H_.eigenvalues()(0);

  for (uint i = 0; i < window_; ++i)
  {
    // Matriz temporária usando Rank-1 update
    H_temp_ = H_ - (zspace_stack_.row(i).transpose() * zspace_stack_.row(i)) + zTz_new_;

    solver_H_.compute(H_temp_);
    min_lambda_ = solver_H_.eigenvalues()(0);

    // Maximize lowest eigen value
    if (min_lambda_ > lowest_lambda_ + 1e-6)
    {
      lowest_lambda_ = min_lambda_;
      subst_idx_ = i;
    }
  }

  // Replace the least informative sample
  if (subst_idx_ != -1)
  {
    H_ =
      H_ - (zspace_stack_.row(subst_idx_).transpose() * zspace_stack_.row(subst_idx_)) + zTz_new_;
    zspace_stack_.row(subst_idx_) = zspace_new_point_.transpose();
    fint_stack_(subst_idx_) = fint_new_;
  }
  return 0;
}

}  // namespace ros2_impedance_controller
