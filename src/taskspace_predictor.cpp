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

  gravityN_ = Eigen::VectorXd::Zero(dof_ * horizon);
  robot_g_ = Eigen::VectorXd::Zero(dof_ * horizon);

  Ak_ = Eigen::MatrixXd::Identity(kStateSpaceDim, kStateSpaceDim);
  Bk_ = Eigen::MatrixXd::Zero(kStateSpaceDim, dof_);

  AN_.resize(horizon);
  BN_.resize(horizon);
  for (size_t i = 0; i < horizon; i++)
  {
    AN_[i] = Eigen::MatrixXd::Zero(kStateSpaceDim, kStateSpaceDim);
    BN_[i] = Eigen::MatrixXd::Zero(kStateSpaceDim, dof_);
  }

  F_ = Eigen::MatrixXd::Zero(kStateSpaceDim * horizon, kStateSpaceDim);
  G_ = Eigen::MatrixXd::Zero(kStateSpaceDim * horizon, dof_ * horizon);

  robot_ddq_.resize(robot_.nv);
  robot_dq_.resize(robot_.nv);
  robot_q_.resize(robot_.nq);
  robot_ddq_.setZero();
  robot_dq_.setZero();
  robot_q_.setZero();
}

void TaskspacePredictor::predict(Eigen::VectorXd q, Eigen::VectorXd v, Eigen::VectorXd tau)
{
  // Iteration '0' with tau_0 = tau
  robot_ddq_ = pinocchio::aba(robot_, *data_.get(), q, v, tau);
  robot_dq_ = v + timestep_ * robot_ddq_;
  robot_q_ = pinocchio::integrate(robot_, q, timestep_ * robot_dq_);
  robot_g_.segment(0, dof_) = pinocchio::computeGeneralizedGravity(robot_, *data_.get(), robot_q_);
  update_inertia();

  update_jacobian();
  assemble_Jn(0);

  update_Bk(0);
  update_Ak(0);

  update_coriolis();
  assemble_Cn(0);

  for (size_t k = 1; k < horizon_; ++k)
  {
    // Sample and hold the torque input:
    robot_ddq_ = pinocchio::aba(robot_, *data_.get(), robot_q_, robot_dq_, tau);
    robot_dq_ = robot_dq_ + timestep_ * robot_ddq_;
    robot_q_ = pinocchio::integrate(robot_, robot_q_, timestep_ * robot_dq_);
    robot_g_.segment(k * dof_, dof_) =
      pinocchio::computeGeneralizedGravity(robot_, *data_.get(), robot_q_);
    update_inertia();

    update_jacobian();
    assemble_Jn(k);

    update_Bk(k);
    update_Ak(k);

    update_coriolis();
    assemble_Cn(k);
  }

  // Assemble gravity stack:
  gravityN_ = jacobianN_ * robot_g_;
  // Assemble state space stacks:
  assemble_F();
  assemble_G();
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

void TaskspacePredictor::assemble_F()
{
  F_.block(0, 0, kStateSpaceDim, kStateSpaceDim) = AN_[0];
  for (size_t i = 1; i < horizon_; i++)
  {
    F_.block(kStateSpaceDim * i, 0, kStateSpaceDim, kStateSpaceDim) =
      AN_[i] * F_.block(kStateSpaceDim * (i - 1), 0, kStateSpaceDim, kStateSpaceDim);
  }
}

void TaskspacePredictor::assemble_G()
{
  // i: row, j: column
  for (size_t j = 0; j < horizon_; j++)
  {
    // fill the "diagonal":
    G_.block(kStateSpaceDim * j, dof_ * j, kStateSpaceDim, dof_) = BN_[j];
    // fill the lower triangular part:
    if (j < (horizon_ - 1))
    {
      for (size_t i = j + 1; i < horizon_; i++)
      {
        G_.block(kStateSpaceDim * i, dof_ * j, kStateSpaceDim, dof_) =
          AN_[i] * G_.block(kStateSpaceDim * (i - 1), dof_ * j, kStateSpaceDim, dof_);
      }
    }
  }
}

void TaskspacePredictor::update_jacobian()
{
  pinocchio::computeFrameJacobian(
    robot_, *data_.get(), robot_q_, ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

  pinocchio::getFrameJacobianTimeVariation(
    robot_, *data_.get(), ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_dt_);

  jacobian_inv_ = jacobian_.completeOrthogonalDecomposition().pseudoInverse();
}

void TaskspacePredictor::update_coriolis()
{
  pinocchio::computeCoriolisMatrix(robot_, *data_.get(), robot_q_, robot_dq_);
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

void TaskspacePredictor::update_Ak(std::size_t n)
{
  Ak_.block(0, kCartesianDim, kCartesianDim, kCartesianDim) =
    timestep_ * Eigen::MatrixXd::Identity(kCartesianDim, kCartesianDim);

  Ak_.block(kCartesianDim, kCartesianDim, kCartesianDim, kCartesianDim) =
    Eigen::MatrixXd::Identity(kCartesianDim, kCartesianDim) -
    (Bk_.block(kCartesianDim, 0, kCartesianDim, dof_) * data_->C - timestep_ * jacobian_dt_) *
      jacobian_inv_;
  // Above we are reusing Bk_ = (MJ^(-1))^-1 term in Ak_, for computational optimization.
  AN_[n] = Ak_;
}

void TaskspacePredictor::update_Bk(std::size_t n)
{
  Bk_.block(kCartesianDim, 0, kCartesianDim, dof_) =
    timestep_ * (data_->M * jacobian_inv_).colPivHouseholderQr().inverse();

  BN_[n] = Bk_;
}

Eigen::MatrixXd TaskspacePredictor::get_Jn() { return jacobianN_; }

Eigen::MatrixXd TaskspacePredictor::get_Cn() { return coriolisN_; }

Eigen::MatrixXd TaskspacePredictor::get_gn() { return gravityN_; }

Eigen::MatrixXd TaskspacePredictor::get_F_matrix() { return F_; }

Eigen::MatrixXd TaskspacePredictor::get_G_matrix() { return G_; }

TaskspacePredictor::~TaskspacePredictor() { data_.reset(); }
}  // namespace ros2_impedance_controller
