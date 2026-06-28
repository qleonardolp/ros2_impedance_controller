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

#include "ros2_impedance_controller/nonlinear_cartesian_controller.hpp"

namespace ros2_impedance_controller
{
NonlinearCartesianController::NonlinearCartesianController() : ImpedanceControllerBase() {}

void NonlinearCartesianController::declare_parameters()
{
  param_listener_ = std::make_shared<::nonlinear_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn NonlinearCartesianController::read_parameters()
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

  joint_names_ = params_.joints;
  base_link_name_ = params_.base_link;
  end_effector_link_name_ = params_.interaction_link;
  urdf_package_ = params_.urdf_package;
  urdf_relative_path_ = params_.urdf_relative_path;
  visualize_reference_ = params_.visualize_reference;
  has_effort_states_ = params_.has_effort_states;
  spring_force_limit_ = params_.spring_force_limit;

  if (params_.stiffness.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'stiffness' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  nominal_stiffness_ = Vector6d(params_.stiffness.data()).asDiagonal();  // Nominal stiffness

  if (params_.damping.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'damping' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  nominal_damping_ = Vector6d(params_.damping.data()).asDiagonal();  // Nominal damping
  desired_damping_ = nominal_damping_;

  return controller_interface::CallbackReturn::SUCCESS;
}

void NonlinearCartesianController::custom_configuration()
{
  steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  status_msg_.data.resize(25, 0.0);

  // Initialize dynamic Eigen members
  tau_desired_.resize(get_dof());
  jacobian_pinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
  jacobianT_pinv_ = Eigen::MatrixXd::Zero(kCartesianDim, get_dof());
  jsim_jpinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
  jsim_jpinv_dj_ = Eigen::MatrixXd::Zero(get_dof(), get_dof());
  jacobian_dt_ = Eigen::MatrixXd::Zero(kCartesianDim, get_dof());
}

void NonlinearCartesianController::custom_activation()
{
  // Dynamic size members (joint space dim)
  tau_desired_.setZero();

  // PH related
  impedance_expected_input_.setZero();
  hamiltonian_filtered_ = 0.0;
  hamiltonian_last_ = 0.0;

  observations_.setOnes();
  observer_A_.setOnes();
  vel_series_.setZero();
  imp_series_.setZero();
}

controller_interface::CallbackReturn NonlinearCartesianController::update_effort_commands()
{
  update_start_ = steady_clock_->now();
  jacobian_pinv_ = jacobian_.completeOrthogonalDecomposition().pseudoInverse();
  jacobianT_pinv_ = jacobian_.transpose().colPivHouseholderQr().inverse();

  accel_deviation_.noalias() = accel_;

  jsim_jpinv_ = robot_data_->M * jacobian_pinv_;
  actual_inertia_ = jacobianT_pinv_ * jsim_jpinv_;

  // observe_inertia_and_disturbance();

  update_stiffness();
  // update_damping();

  desired_inertia_ = actual_inertia_;  // to compute Hamiltonian function

  impedance_wrench_.noalias() =
    desired_stiffness_ * pose_deviation_ + desired_damping_ * twist_deviation_;
  tau_desired_.noalias() = -jacobian_.transpose() * impedance_wrench_;

  if (params_.gravity_compensation)
  {
    tau_desired_ += robot_data_->g;
  }

  effort_commands_ = cmd_lpf_alpha_ * tau_desired_ + (1.0 - cmd_lpf_alpha_) * effort_commands_;

  compute_hamiltonian();

  update_end_ = steady_clock_->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

void NonlinearCartesianController::publish_status()
{
  // Robot Hamiltonian - Impedance Hamiltonian
  status_msg_.data[0] = robot_data_->mechanical_energy - hamiltonian_filtered_;
  // Commands input power
  status_msg_.data[1] = robot_dq_.transpose() * effort_commands_;
  // Interaction power using estimated interaction wrench
  status_msg_.data[2] = twist_.transpose() * estimated_wrench_;
  // Impedance Hamiltonian
  status_msg_.data[3] = hamiltonian_filtered_;
  // Impedance Hamiltonian derivative
  status_msg_.data[4] = actual_inertia_(2, 2);  // trying to visualize m_zz
  // Impedance I/O power
  status_msg_.data[5] = twist_deviation_.transpose() * impedance_wrench_;

  // Impedance space:
  status_msg_.data[6] = pose_deviation_(0);
  status_msg_.data[7] = pose_deviation_(1);
  status_msg_.data[8] = pose_deviation_(2);
  status_msg_.data[9] = pose_deviation_(3);
  status_msg_.data[10] = pose_deviation_(4);
  status_msg_.data[11] = pose_deviation_(5);

  status_msg_.data[12] = twist_deviation_(0);
  status_msg_.data[13] = twist_deviation_(1);
  status_msg_.data[14] = twist_deviation_(2);
  status_msg_.data[15] = twist_deviation_(3);
  status_msg_.data[16] = twist_deviation_(4);
  status_msg_.data[17] = twist_deviation_(5);

  status_msg_.data[18] = estimated_wrench_(0);
  status_msg_.data[19] = estimated_wrench_(1);
  status_msg_.data[20] = estimated_wrench_(2);
  status_msg_.data[21] = desired_stiffness_.diagonal()(0);
  status_msg_.data[22] = desired_stiffness_.diagonal()(1);
  status_msg_.data[23] = desired_stiffness_.diagonal()(2);

  status_msg_.data[24] = (update_end_ - update_start_).seconds();

  status_rt_publisher_->try_publish(status_msg_);
}

void NonlinearCartesianController::compute_hamiltonian()
{
  // When inertia shaping is disabled, desired_inertia_ is the actual_inertia_.
  hamiltonian_ = twist_deviation_.transpose() * desired_inertia_ * twist_deviation_;
  hamiltonian_ += pose_deviation_.transpose() * desired_stiffness_ * pose_deviation_;
  hamiltonian_ = 0.5 * hamiltonian_;

  hamiltonian_filtered_ =
    cmd_lpf_alpha_ * hamiltonian_ + (1.0 - cmd_lpf_alpha_) * hamiltonian_filtered_;
  hamiltonian_derivative_ = (hamiltonian_filtered_ - hamiltonian_last_) / delta_t_;
  hamiltonian_last_ = hamiltonian_filtered_;
}

void NonlinearCartesianController::update_stiffness()
{
  // Update stiffness: K(dx) := k_0 / sqrt(1 + (k_0*dx/F_max)^2)
  desired_stiffness_.diagonal() = nominal_stiffness_.diagonal().array().cwiseProduct(
    (((nominal_stiffness_ * pose_deviation_).array() / spring_force_limit_).square() + 1.0)
      .rsqrt());
}

void NonlinearCartesianController::update_damping()
{
  desired_damping_ = nominal_damping_;

  pinocchio::cholesky::decompose(robot_model_, *robot_data_.get());
  pinocchio::cholesky::computeMinv(robot_model_, *robot_data_.get());

  pinocchio::getFrameJacobianTimeVariation(
    robot_model_, *robot_data_.get(), end_effector_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
    jacobian_dt_);

  jsim_jpinv_dj_ = jsim_jpinv_ * jacobian_dt_;

  // Partitioning check
  is_dissipative_ = true;
  for (size_t i = 0; i < get_dof(); i++)
  {
    auto mInv_col = robot_data_->Minv.col(i);
    is_dissipative_ &= desired_damping_.diagonal()(i) >
                       -jsim_jpinv_dj_.row(i).dot(mInv_col) / jacobian_.row(i).dot(mInv_col);
  }

  // Phase space (q,p) divergence:
  divergence_ = -((jsim_jpinv_dj_ + desired_damping_ * jacobian_) * robot_data_->Minv).trace();
}

void NonlinearCartesianController::observe_inertia_and_disturbance()
{
  // roll along 4 samples
  for (size_t k = 3; k > 0; --k)
  {
    vel_series_(k) = vel_series_(k - 1);
    imp_series_(k) = imp_series_(k - 1);
  }
  vel_series_(0) = twist_(0);             // 'x' only
  imp_series_(0) = impedance_wrench_(0);  // 'x' only
  // Remember: after that we have
  // (0) -> k+3
  // (1) -> k+2
  // (2) -> k+1
  // (3) -> k
  observer_A_.col(0) = vel_series_.tail<3>();
  observer_A_.col(1) = imp_series_.tail<3>();

  observations_ = observer_A_.colPivHouseholderQr().solve(vel_series_.head<3>());
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::NonlinearCartesianController,
  controller_interface::ControllerInterface)
