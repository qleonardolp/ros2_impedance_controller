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
  jacobianT_inv_ = Eigen::MatrixXd::Zero(kCartesianDim, dof_);
  jacobianN_ = Eigen::MatrixXd::Zero(kCartesianDim * horizon, dof_ * horizon);

  jacobian_dt_ = Eigen::MatrixXd::Zero(kCartesianDim, dof_);
  jacobian_inv_ = Eigen::MatrixXd::Zero(dof_, kCartesianDim);

  coriolisN_ = Eigen::MatrixXd::Zero(kCartesianDim * horizon, kCartesianDim * horizon);
  task_coriolis_.setIdentity();

  Ak_ = Eigen::MatrixXd::Zero(kCartesianDim * 2, kCartesianDim * 2);
  Bk_ = Eigen::MatrixXd::Zero(kCartesianDim * 2, dof_);

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
  update_inertia();

  update_jacobian();
  assemble_Jn(0);

  update_coriolis();
  assemble_Cn(0);

  robot_Q_.segment(0, dof_) = robot_q_;

  for (size_t k = 1; k < horizon_; ++k)
  {
    // Next to the 0-th iteration, the prediction is open loop, then the tau vector is zero.
    robot_ddq_ = pinocchio::aba(robot_, *data_.get(), robot_q_, robot_dq_, zero_tau_);

    // Sample and hold the torque input:
    // robot_ddq_ = pinocchio::aba(robot_, *data_.get(), robot_q_, robot_dq_, tau);
    robot_dq_ = robot_dq_ + timestep_ * robot_ddq_;
    robot_q_ = pinocchio::integrate(robot_, robot_q_, timestep_ * robot_dq_);
    update_inertia();

    update_jacobian();
    assemble_Jn(k);

    update_coriolis();
    assemble_Cn(k);

    robot_Q_.segment(k * dof_, dof_) = robot_q_;
  }
}

void TaskspacePredictor::assemble_Jn(std::size_t n)
{
  jacobianT_inv_ = jacobian_.transpose().colPivHouseholderQr().inverse();
  jacobianN_.block(kCartesianDim * n, dof_ * n, kCartesianDim, dof_) = jacobianT_inv_;
}

void TaskspacePredictor::assemble_Cn(std::size_t n)
{
  coriolisN_.block(kCartesianDim * n, kCartesianDim * n, kCartesianDim, kCartesianDim) =
    task_coriolis_;
}

void TaskspacePredictor::update_jacobian()
{
  pinocchio::computeFrameJacobian(
    robot_, *data_.get(), robot_q_, ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

  pinocchio::getFrameJacobianTimeVariation(
    robot_, *data_.get(), ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_dt_);
}

void TaskspacePredictor::update_coriolis()
{
  pinocchio::computeCoriolisMatrix(robot_, *data_.get(), robot_q_, robot_dq_);
  jacobian_inv_ = jacobian_.completeOrthogonalDecomposition().pseudoInverse();
  task_coriolis_ =
    jacobianT_inv_ * (data_->C - data_->M * jacobian_inv_ * jacobian_dt_) * jacobian_inv_;
}

void TaskspacePredictor::update_inertia()
{
  pinocchio::crba(robot_, *data_.get(), robot_q_);  // compute M
  data_->M.triangularView<Eigen::StrictlyLower>() =
    data_->M.transpose().triangularView<Eigen::StrictlyLower>();
  // Minv is already computed by pinocchio::aba
  data_->Minv.triangularView<Eigen::StrictlyLower>() =
    data_->Minv.transpose().triangularView<Eigen::StrictlyLower>();
}

Eigen::MatrixXd TaskspacePredictor::get_Jn() { return jacobianN_; }

Eigen::MatrixXd TaskspacePredictor::get_Cn() { return coriolisN_; }

Eigen::VectorXd TaskspacePredictor::get_positions() { return robot_Q_; }

TaskspacePredictor::~TaskspacePredictor() { data_.reset(); }
}  // namespace ros2_impedance_controller
