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

#include "ros2_impedance_controller/taskspace_predictor.hpp"

namespace ros2_impedance_controller
{
TaskspacePredictor::TaskspacePredictor(
  double timestep, uint horizon, std::string ee_frame, pinocchio::Model robot)
: timestep_(timestep), horizon_(horizon), robot_(robot)
{
  data_ = std::make_shared<pinocchio::Data>(robot_);

  if (!robot_.existFrame(ee_frame))
  {
    std::cout << "Could not find frame '" << ee_frame << "' on model " << robot_.name << std::endl;
  }
  ee_frame_ = robot_.getFrameId(ee_frame);
  dof_ = robot_.nq;

  jacobian_ = Eigen::MatrixXd::Zero(kCartesianDim, dof_);

  robot_ddq_.resize(robot_.nv);
  robot_dq_.resize(robot_.nv);
  robot_q_.resize(robot_.nq);
  zero_tau_.resize(robot.nq);

  robot_Q_.resize(dof_ * horizon);

  robot_ddq_.setZero();
  robot_dq_.setZero();
  robot_q_.setZero();
  zero_tau_.setZero();
}

void TaskspacePredictor::predict(Eigen::VectorXd q, Eigen::VectorXd v, Eigen::VectorXd tau)
{
  // Iteration '0' with tau_0 = tau
  robot_ddq_ = pinocchio::aba(robot_, *data_.get(), q, v, tau);
  robot_dq_ = v + timestep_ * robot_ddq_;
  robot_q_ = pinocchio::integrate(robot_, q, timestep_ * robot_dq_);
  // TODO(@me): get data_->Minv, J(q), data_->C, dJ ...

  robot_Q_.segment(0, dof_) = robot_q_;

  for (size_t k = 1; k < horizon_; ++k)
  {
    // Next to the 0-th iteration, the prediction is open loop,
    // then the tau vector is zero.
    robot_ddq_ = pinocchio::aba(robot_, *data_.get(), robot_q_, robot_dq_, zero_tau_);
    robot_dq_ = robot_dq_ + timestep_ * robot_ddq_;
    robot_q_ = pinocchio::integrate(robot_, robot_q_, timestep_ * robot_dq_);
    // TODO(@me): get data_->Minv, J(q), data_->C, dJ ...
    pinocchio::computeFrameJacobian(
      robot_, *data_.get(), robot_q_, ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

    robot_Q_.segment(k * dof_, dof_) = robot_q_;
    // pinocchio::computeMinverse(robot_, *data_.get(), robot_q_); // already computed by ::aba
  }
}

Eigen::VectorXd TaskspacePredictor::get_positions() { return robot_Q_; }

TaskspacePredictor::~TaskspacePredictor() { data_.reset(); }
}  // namespace ros2_impedance_controller
