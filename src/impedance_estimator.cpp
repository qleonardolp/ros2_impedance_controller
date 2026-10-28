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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <qpOASES/QProblemB.hpp>

namespace ros2_impedance_controller
{
ImpedanceEstimator::ImpedanceEstimator(uint window_size, uint axis, double delta_t)
: window_(window_size), axis_(axis), delta_t_(delta_t)
{
  if (window_size < kParamsDim) window_ = kParamsDim;

  // Concurrent Learning
  zspace_record_ = std::make_shared<SlidingWindow>(3, 3);
  zspace_rank_checking_.setIdentity();
  record_updated_ = false;

  zspace_record_->push(Eigen::Vector3d(1.0, 0.0, 0.0));
  zspace_record_->push(Eigen::Vector3d(0.0, 1.0, 0.0));
  zspace_record_->push(Eigen::Vector3d(0.0, 0.0, 1.0));

  state_counter_ = 0;
  fsm_last_state_ = State::NULLSPACE;
  fsm_current_state_ = State::NULLSPACE;

  cputime_ = delta_t * 0.2;  // timestep 80% margin

  action_dim_ = kParamsDim;
  constraints_dim_ = action_dim_;
  nWSR_ = 5 * (action_dim_ + constraints_dim_);  // TODO(@qleonardolp): validate reference

  // Instantiate and configure SQProblem class ptr:
  sqproblem_ = std::make_shared<qpOASES::SQProblem>(action_dim_, constraints_dim_);
  sqp_options_.setToMPC();  // options to minimum solution time
  sqp_options_.printLevel = qpOASES::PrintLevel::PL_NONE;
  sqproblem_->setOptions(sqp_options_);

  // QP Eigen members initialization:
  H_qp_ = Eigen::MatrixXd::Identity(action_dim_, action_dim_);
  A_qp_ = Eigen::MatrixXd::Identity(constraints_dim_, action_dim_);
  g_qp_ = Eigen::MatrixXd::Zero(1, action_dim_);
  // QP action space bounds:
  double slew_limit = 0.05;
  du_max_ = Eigen::VectorXd::Ones(kParamsDim);
  du_max_ << 500.0, 50.0, 0.50;
  ubA_qp_ = Eigen::VectorXd::Ones(constraints_dim_) * slew_limit;
  lbA_qp_ = -ubA_qp_;

  // Cost function-specific initialization:
  u_qp_ = Eigen::VectorXd::Zero(action_dim_);
  b_qp_ = Eigen::VectorXd::Zero(window_);
  V_qp_ = Eigen::MatrixXd::Zero(window_, action_dim_);

  // Set constant matrices:
  Q_qp_ = Eigen::MatrixXd::Identity(action_dim_, action_dim_);
  ub_qp_ = Eigen::VectorXd::Zero(action_dim_);
  lb_qp_ = Eigen::VectorXd::Zero(action_dim_);

  // Set bounds
  ub_qp_ << 10000.0, 100.0, 40.0;
  lb_qp_ << 1e-3, -100.0, 1e-3;

  init_sqproblem();
}

void ImpedanceEstimator::init_sqproblem()
{
  update_QP();
  // QP initialization
  int init_nWSR = 10;
  sqp_ret_ = sqproblem_->init(
    H_qp_.data(), g_qp_.data(), A_qp_.data(), lb_qp_.data(), ub_qp_.data(), lbA_qp_.data(),
    ubA_qp_.data(), init_nWSR);
}

int ImpedanceEstimator::update_state(
  const Vector6d deviation, const Vector6d twist, const Vector6d accel, const Vector6d wrench)
{
  // Check data integrity
  if (deviation.hasNaN() || twist.hasNaN() || accel.hasNaN() || wrench.hasNaN()) return 1;

  // TODO(@qleonardolp): LPF accel

  uint_fast8_t bitmask = ((abs(deviation(axis_)) < kDeviationThreshold) << 3) |
                         ((abs(twist(axis_)) < kTwistDevThreshold) << 2) |
                         ((abs(accel(axis_)) < kAccelDevThreshold) << 1) |
                         (abs(wrench(axis_)) < kIntEffortThreshold);
  fsm_current_state_ = StateLUT[bitmask];

  if (fsm_current_state_ == fsm_last_state_)
  {
    state_counter_++;
    // update Cost function and QP matrices
    for (size_t n = window_ - 1; n > 0; --n)
    {
      V_qp_.row(n) = V_qp_.row(n - 1);
      b_qp_(n) = b_qp_(n - 1);
    }
    V_qp_.row(0) << deviation(axis_), twist(axis_), accel(axis_);
    b_qp_(0) = wrench(axis_);
  }

  // Update bounds vector accordingly
  (this->*bounds_table_[get_fsm_state()])();

  // Update data record for CL:
  record_updated_ = false;
  zspace_rank_checking_ = zspace_record_->get_buffer();
  zspace_new_point_ << deviation(axis_), twist(axis_), accel(axis_);
  zspace_rank_checking_.row(0) = zspace_new_point_;

  zspace_LU_.compute(zspace_rank_checking_);  // compute LU decomposition

  if (zspace_LU_.rank() >= 3)
  {
    record_updated_ = true;
    zspace_record_->push(zspace_new_point_);
  }

  fsm_last_state_ = fsm_current_state_;
  return 0;
}

void ImpedanceEstimator::update_QP()
{
  H_qp_.noalias() = 2 * V_qp_.transpose() * Q_qp_ * V_qp_;
  g_qp_.noalias() = 2 * V_qp_.transpose() * Q_qp_ * b_qp_;
}

int ImpedanceEstimator::update()
{
  // hotstart method output the taken nWSR and cputime values in place.
  // Then we must reset those values at every loop iteration
  cputime_qp_ = cputime_;
  nWSR_qp_ = nWSR_;

  if (state_counter_ % window_ != 0) return 0;

  state_counter_ = 0;

  // TODO(@me): subtract the average from each column and b

  /*
  if (debug_)
  {
    std::cout << "V_qp_: " << std::fixed << std::setprecision(2) << V_qp_ << std::endl;
    debug_ = false;
  }
  */

  // update inequality constraints
  lbA_qp_.head(kParamsDim) = u_qp_.head(kParamsDim) - du_max_;
  ubA_qp_.head(kParamsDim) = u_qp_.head(kParamsDim) + du_max_;

  /*
  update_QP();
  // Solve SQProblem
  sqp_ret_ = sqproblem_->hotstart(
    H_qp_.data(), g_qp_.data(), A_qp_.data(), lb_qp_.data(), ub_qp_.data(), lbA_qp_.data(),
    ubA_qp_.data(), nWSR_qp_, &cputime_qp_);
  sqproblem_->getPrimalSolution(u_qp_.data());

  if (sqp_ret_ != qpOASES::returnValue::RET_QP_SOLVED) return 2;  // Iteration failed.
  */

  return 0;
}

void ImpedanceEstimator::set_nullspace_bounds()
{
  ub_qp_ << u_qp_;
  lb_qp_ << u_qp_;
}

void ImpedanceEstimator::set_free_motion_bounds()
{
  ub_qp_ << 10000.0, 100.0, 40.0;
  lb_qp_ << 1e-3, -100.0, 1e-3;
}

void ImpedanceEstimator::set_static_int_bounds()
{
  ub_qp_ << 10000.0, u_qp_(1), u_qp_(2);
  lb_qp_ << 1e-3, u_qp_(1), u_qp_(2);
}

void ImpedanceEstimator::set_equilibrium_bounds()
{
  ub_qp_ << u_qp_(0), u_qp_(1), 40.0;
  lb_qp_ << u_qp_(0), u_qp_(1), 1e-3;
}

}  // namespace ros2_impedance_controller
