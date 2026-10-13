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
: window_(window_size), axis_(axis)
{
  if (window_size < kParamsDim) window_ = kParamsDim;

  cputime_ = delta_t * 0.5;  // timestep 50% margin

  action_dim_ = kParamsDim * window_;
  constraints_dim_ = action_dim_;
  nWSR_ = 5 * (action_dim_ + constraints_dim_);  // TODO(@qleonardolp): validate reference

  // Instantiate and configure SQProblem class ptr:
  sqproblem_ = std::make_shared<qpOASES::SQProblem>(action_dim_, constraints_dim_);
  sqp_options_.setToMPC();  // options to minimum solution time
  sqp_options_.printLevel = qpOASES::PrintLevel::PL_NONE;
  sqproblem_->setOptions(sqp_options_);

  // QP Eigen members initialization:
  H_qp_ = Eigen::MatrixXd::Identity(action_dim_, action_dim_);
  A_qp_ = Eigen::MatrixXd::Zero(constraints_dim_, action_dim_);
  g_qp_ = Eigen::MatrixXd::Zero(1, action_dim_);
  // QP action space bounds:
  du_max_ = Eigen::VectorXd::Ones(kParamsDim) * 0.9;
  ubA_qp_ = Eigen::VectorXd::Ones(constraints_dim_) * 0.9;
  lbA_qp_ = -ubA_qp_;
  ub_qp_ = Eigen::VectorXd::Ones(action_dim_) * 100.0;
  lb_qp_ = -ub_qp_;

  // Cost function-specific initialization:
  u_qp_ = Eigen::VectorXd::Zero(action_dim_);
  b_qp_ = Eigen::VectorXd::Zero(window_);
  V_qp_ = Eigen::MatrixXd::Zero(window_, action_dim_);
  // Set constant matrices:
  Q_qp_ = Eigen::MatrixXd::Identity(action_dim_, action_dim_);
}

void ImpedanceEstimator::init_sqproblem()
{
  assemble_Aqp();

  // QP initialization
  int init_nWSR = 10;
  sqp_ret_ = sqproblem_->init(
    H_qp_.data(), g_qp_.data(), A_qp_.data(), lb_qp_.data(), ub_qp_.data(), lbA_qp_.data(),
    ubA_qp_.data(), init_nWSR);
}

int ImpedanceEstimator::update(const Vector6d accel, const Vector6d twist, const Vector6d wrench)
{
  // hotstart method output the taken nWSR and cputime values in place.
  // Then we must reset those values at every loop iteration
  cputime_qp_ = cputime_;
  nWSR_qp_ = nWSR_;

  // Check data integrity
  if (accel.hasNaN() || twist.hasNaN() || wrench.hasNaN()) return 1;

  // update inequality constraints
  lbA_qp_.head(kParamsDim) = u_qp_.head(kParamsDim) - du_max_;
  ubA_qp_.head(kParamsDim) = u_qp_.head(kParamsDim) + du_max_;

  // update Cost function and QP matrices
  update_QP();

  // Solve SQProblem
  sqp_ret_ = sqproblem_->hotstart(
    H_qp_.data(), g_qp_.data(), A_qp_.data(), lb_qp_.data(), ub_qp_.data(), lbA_qp_.data(),
    ubA_qp_.data(), nWSR_qp_, &cputime_qp_);
  sqproblem_->getPrimalSolution(u_qp_.data());

  if (sqp_ret_ != qpOASES::returnValue::RET_QP_SOLVED)  // Iteration failed.
  {
    return 2;
  }

  /*
  if (debug_)
  {
    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "H_qp_: " << std::fixed << std::setprecision(2) << H_qp_);
    debug_ = false;
  }
  */
  return 0;
}

void ImpedanceEstimator::assemble_Aqp() {}

void ImpedanceEstimator::update_QP()
{
  // b = L*F*x_0 - Z*x_{d} - J*g(q)
  // b_qp_.noalias() = L_qp_ * predictor_->get_F_matrix() * task_states_
  // V = L*G + J
  // V_qp_.noalias() = L_qp_ * predictor_->get_G_matrix() + predictor_->get_Jn();

  H_qp_.noalias() = 2 * V_qp_.transpose() * Q_qp_ * V_qp_;
  g_qp_.noalias() = 2 * V_qp_.transpose() * Q_qp_ * b_qp_;
}

}  // namespace ros2_impedance_controller
