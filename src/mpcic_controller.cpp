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

#include "ros2_impedance_controller/mpcic_controller.hpp"

namespace ros2_impedance_controller
{
MPCIController::MPCIController() : ImpedanceControllerBase() {}

void MPCIController::declare_parameters()
{
  param_listener_ = std::make_shared<::mpcic_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn MPCIController::read_parameters()
{
  params_ = param_listener_->get_params();

  if (params_.urdf_package.empty() || params_.urdf_relative_path.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Missing robot URDF package or path params, required for Pinocchio RBD library.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Required by the base class:
  joint_names_ = params_.joints;
  base_link_name_ = params_.base_link;
  end_effector_link_name_ = params_.interaction_link;
  urdf_package_ = params_.urdf_package;
  urdf_relative_path_ = params_.urdf_relative_path;
  visualize_reference_ = params_.visualize_reference;
  has_effort_states_ = params_.has_effort_states;

  // MPC specific:
  timestep_ = params_.timestep;
  cputime_ = 0.001 * 0.5;  // 50% margin
  horizon_ = static_cast<uint8_t>(params_.horizon);

  if (params_.stiffness.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'stiffness' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  desired_stiffness_ = Vector6d(params_.stiffness.data()).asDiagonal();

  if (params_.damping.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'damping' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  desired_damping_ = Vector6d(params_.damping.data()).asDiagonal();

  return controller_interface::CallbackReturn::SUCCESS;
}

void MPCIController::custom_configuration()
{
  dof_ = get_dof();
  status_msg_.data.resize(18, 0.0);
  // Initialize dynamic Eigen members
  tau_desired_.resize(dof_);
  jacobian_pinv_ = Eigen::MatrixXd::Zero(dof_, kCartesianDim);
  jacobianT_pinv_ = Eigen::MatrixXd::Zero(kCartesianDim, dof_);

  predictor_ = std::make_shared<TaskspacePredictor>(
    timestep_, horizon_, end_effector_link_name_, robot_model_);

  constraints_dim_ = static_cast<int>(dof_);
  action_dim_ = constraints_dim_ * horizon_;
  nWSR_ = 5 * (action_dim_ + constraints_dim_);  // TODO(@qleonardolp): validate s.o.t.a. Ref

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
  ub_qp_ = Eigen::MatrixXd::Ones(1, action_dim_) * params_.torque_max;
  lb_qp_ = -ub_qp_;
  // Cost function-specific initialization:
  u_qp_ = Eigen::VectorXd::Zero(action_dim_);
  b_qp_ = Eigen::VectorXd::Zero(kCartesianDim * horizon_);
  V_qp_ = Eigen::MatrixXd::Zero(kCartesianDim * horizon_, action_dim_);
  L_qp_ = Eigen::MatrixXd::Zero(kCartesianDim * horizon_, kStateSpaceDim * horizon_);
  Z_qp_ = L_qp_;

  task_desired_ = Eigen::VectorXd::Zero(kStateSpaceDim * horizon_);

  // Set constant matrices:
  A_qp_.block(0, 0, dof_, dof_) = Eigen::MatrixXd::Identity(dof_, dof_);
  Q_qp_ = Eigen::MatrixXd::Identity(kCartesianDim * horizon_, kCartesianDim * horizon_);
  R_qp_ = Eigen::MatrixXd::Identity(action_dim_, action_dim_);
}

void MPCIController::custom_activation()
{
  tau_desired_.setZero();
  task_states_.setZero();
  impedance_wrench_.setZero();
  estimated_wrench_.setZero();

  // PH related
  hamiltonian_filtered_ = 0.0;
  hamiltonian_last_ = 0.0;

  assemble_Ln();  // initially depends on Kd an Dd only

  // QP initialization
  int init_nWSR = 10;
  sqp_ret_ = sqproblem_->init(
    H_qp_.data(), g_qp_.data(), A_qp_.data(), lb_qp_.data(), ub_qp_.data(), effort_commands_.data(),
    effort_commands_.data(), init_nWSR);

  RCLCPP_INFO(
    get_node()->get_logger(), "QP init status: %i, with nWSR=%i, cputime=%.4f",
    static_cast<int>(sqp_ret_), nWSR_, cputime_);
}

controller_interface::CallbackReturn MPCIController::update_effort_commands()
{
  update_taskstates();
  // hotstart method output the taken nWSR and cputime values in place.
  // Then we must reset those values at every loop iteration
  cputime_qp_ = cputime_;
  nWSR_qp_ = nWSR_;

  // run taskspace predictor
  predictor_->predict(robot_positions_, robot_velocities_, effort_commands_);

  // update Cost function and QP matrices
  update_references();
  update_Ln(predictor_->get_Cn());
  update_QP();

  // Solve SQProblem
  if (sqp_ret_ != qpOASES::returnValue::RET_QP_SOLVED)
  {  // Previous iteration failed. maybe H_qp_ is not positive (semi-)definite ???
    H_qp_ = 2 * R_qp_;
  }

  sqp_ret_ = sqproblem_->hotstart(
    H_qp_.data(), g_qp_.data(), A_qp_.data(), lb_qp_.data(), ub_qp_.data(), effort_commands_.data(),
    effort_commands_.data(), nWSR_qp_, &cputime_qp_);
  sqproblem_->getPrimalSolution(u_qp_.data());

  // use the first action (tau_0)
  // from inspection it looks to be at the end of u_qp_
  tau_desired_ = u_qp_.tail(dof_);
  effort_commands_ = cmd_lpf_alpha_ * tau_desired_ + (1.0 - cmd_lpf_alpha_) * effort_commands_;

  if (debug_)
  {
    RCLCPP_INFO_STREAM(
      get_node()->get_logger(), "u_qp: " << std::fixed << std::setprecision(3) << u_qp_);
    debug_ = false;
  }

  compute_hamiltonian();
  return controller_interface::CallbackReturn::SUCCESS;
}

void MPCIController::assemble_Ln()
{
  for (size_t i = 0; i < horizon_; i++)
  {
    L_qp_.block(kCartesianDim * i, kStateSpaceDim * i, kCartesianDim, kCartesianDim) =
      desired_stiffness_;
    L_qp_.block(
      kCartesianDim * i, kCartesianDim + kStateSpaceDim * i, kCartesianDim, kCartesianDim) =
      desired_damping_;
  }
  Z_qp_ = L_qp_;
}

void MPCIController::update_Ln(Eigen::MatrixXd Cn)
{
  for (size_t i = 0; i < horizon_; i++)
  {
    L_qp_.block(
      kCartesianDim * i, kCartesianDim + kStateSpaceDim * i, kCartesianDim, kCartesianDim) -=
      Cn.block(kCartesianDim * i, kCartesianDim * i, kCartesianDim, kCartesianDim);
  }
}

void MPCIController::update_QP()
{
  // b = L*F*x_0 - Z*x_{d} - J*g(q)
  b_qp_.noalias() = L_qp_ * predictor_->get_F_matrix() * task_states_ - Z_qp_ * task_desired_ -
                    predictor_->get_gn();
  // V = L*G + J
  V_qp_.noalias() = L_qp_ * predictor_->get_G_matrix() + predictor_->get_Jn();

  H_qp_.noalias() = 2 * V_qp_.transpose() * Q_qp_ * V_qp_ + 2 * R_qp_;
  g_qp_.noalias() = 2 * V_qp_.transpose() * Q_qp_ * b_qp_;
}

void MPCIController::update_taskstates()
{
  actual_pose_.head<3>() = robot_data_.get()->oMf[end_effector_frame_].translation();
  actual_pose_.tail<3>() = robot_data_.get()->oMf[end_effector_frame_].rotation().eulerAngles(
    2, 0, 2);  // TODO(@me): validate Euler convention
  task_states_.head<kCartesianDim>() = actual_pose_;
  task_states_.tail<kCartesianDim>() = actual_twist_;  // updated from the base class
}

void MPCIController::update_references()
{
  // roll along the horizon
  for (size_t k = horizon_ - 1; k > 0; --k)
  {
    task_desired_.segment(k * kStateSpaceDim, kStateSpaceDim) =
      task_desired_.segment((k - 1) * kStateSpaceDim, kStateSpaceDim);
  }

  task_desired_(0) = reference_.pose.position.x;
  task_desired_(1) = reference_.pose.position.y;
  task_desired_(2) = reference_.pose.position.z;
  // TODO(@me): resolve x_d Euler angles

  task_desired_(6) = reference_.pose_twist.linear.x;
  task_desired_(7) = reference_.pose_twist.linear.y;
  task_desired_(8) = reference_.pose_twist.linear.z;
  task_desired_(9) = reference_.pose_twist.angular.x;
  task_desired_(10) = reference_.pose_twist.angular.y;
  task_desired_(11) = reference_.pose_twist.angular.z;
}

void MPCIController::publish_status()
{
  // Robot Hamiltonian - Impedance Hamiltonian
  status_msg_.data[0] = robot_data_->mechanical_energy - hamiltonian_filtered_;
  // Commands input power
  status_msg_.data[1] = robot_velocities_.transpose() * effort_commands_;
  // Interaction power using estimated interaction wrench
  status_msg_.data[2] = actual_twist_.transpose() * estimated_wrench_;
  // Impedance Hamiltonian
  status_msg_.data[3] = hamiltonian_filtered_;
  // Impedance Hamiltonian derivative
  status_msg_.data[4] = hamiltonian_derivative_;
  // Impedance I/O power
  status_msg_.data[5] = twist_deviation_.transpose() * impedance_wrench_;

  status_msg_.data[6] = actual_pose_(0);
  status_msg_.data[7] = actual_pose_(1);
  status_msg_.data[8] = actual_pose_(2);
  status_msg_.data[9] = actual_pose_(3);
  status_msg_.data[10] = actual_pose_(4);
  status_msg_.data[11] = actual_pose_(5);

  status_msg_.data[12] = tau_desired_(0);
  status_msg_.data[13] = tau_desired_(1);

  status_msg_.data[15] = cputime_qp_;              // CPU time spent for QP solution
  status_msg_.data[16] = sqproblem_->getObjVal();  // Objective function value
  // status_msg_.data[16] = nWSR_qp_;                    // Number of performed WSR
  status_msg_.data[17] = static_cast<int>(sqp_ret_);  // check QP status

  status_rt_publisher_->try_publish(status_msg_);
}

void MPCIController::phase_space_diagnostics()
{
  status_msg_.data[0] = pose_deviation_(0);
  status_msg_.data[1] = pose_deviation_(1);
  status_msg_.data[2] = pose_deviation_(2);
  // Using the quaternion vector as the angles
  status_msg_.data[3] = pose_deviation_(3);
  status_msg_.data[4] = pose_deviation_(4);
  status_msg_.data[5] = pose_deviation_(5);

  status_msg_.data[6] = twist_deviation_(0);
  status_msg_.data[7] = twist_deviation_(1);
  status_msg_.data[8] = twist_deviation_(2);
  status_msg_.data[9] = twist_deviation_(3);
  status_msg_.data[10] = twist_deviation_(4);
  status_msg_.data[11] = twist_deviation_(5);

  robot_data_->Minv.triangularView<Eigen::StrictlyLower>() =
    robot_data_->Minv.transpose().triangularView<Eigen::StrictlyLower>();

  // Phase space (q,p) divergence
  // First term:
  // status_msg_.data[12] = -(jsim_jpinv_dj_ * robot_data_->Minv).trace();

  // TODO(@qleonardolp) investigate the trace for non square matrices (nDoF < m)
  // Eigen::DiagonalMatrix<double, Eigen::Dynamic, kCartesianDim> damping(
  //   desired_damping_.diagonal().head<dof_>());

  // Second term:
  status_msg_.data[13] = -(desired_damping_ * jacobian_ * robot_data_->Minv).trace();
}

void MPCIController::compute_hamiltonian()
{
  jacobian_pinv_ = jacobian_.completeOrthogonalDecomposition().pseudoInverse();
  jacobianT_pinv_ = jacobian_.transpose().colPivHouseholderQr().inverse();

  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
    robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();
  desired_inertia_ = jacobianT_pinv_ * robot_data_->M * jacobian_pinv_;

  // When inertia shaping is disabled, desired_inertia_ is the actual_inertia_.
  hamiltonian_ = twist_deviation_.transpose() * desired_inertia_ * twist_deviation_;
  hamiltonian_ += pose_deviation_.transpose() * desired_stiffness_ * pose_deviation_;
  hamiltonian_ = 0.5 * hamiltonian_;

  hamiltonian_filtered_ =
    cmd_lpf_alpha_ * hamiltonian_ + (1.0 - cmd_lpf_alpha_) * hamiltonian_filtered_;
  hamiltonian_derivative_ = (hamiltonian_filtered_ - hamiltonian_last_) / delta_t_;
  hamiltonian_last_ = hamiltonian_filtered_;

  // to status:
  impedance_wrench_.noalias() =
    (desired_stiffness_ * pose_deviation_ + desired_damping_ * twist_deviation_) * (-1);

  if (has_effort_states_)
  {
    estimated_wrench_.noalias() = jacobianT_pinv_ * (robot_efforts_ - effort_commands_);
  }
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::MPCIController, controller_interface::ControllerInterface)
