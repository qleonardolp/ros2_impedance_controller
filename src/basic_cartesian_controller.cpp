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

#include "ros2_impedance_controller/basic_cartesian_controller.hpp"

namespace ros2_impedance_controller
{
BasicCartesianController::BasicCartesianController() : ImpedanceControllerBase() {}

void BasicCartesianController::declare_parameters()
{
  param_listener_ = std::make_shared<::cartesian_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn BasicCartesianController::read_parameters()
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

void BasicCartesianController::custom_configuration()
{
  status_msg_.data.resize(24, 0.0);
  // Initialize dynamic Eigen members
  tau_desired_.resize(get_dof());
  accel_feedforward_.resize(get_dof());
  jacobian_pinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
  jacobianT_pinv_ = Eigen::MatrixXd::Zero(kCartesianDim, get_dof());
  jsim_jpinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
  jacobian_dt_ = Eigen::MatrixXd::Zero(kCartesianDim, get_dof());
}

void BasicCartesianController::custom_activation()
{
  // Dynamic size members (joint space dim)
  accel_feedforward_.setZero();
  tau_desired_.setZero();

  impedance_wrench_.setZero();
  estimated_wrench_.setZero();

  // PH related
  impedance_expected_input_.setZero();
  hamiltonian_filtered_ = 0.0;
  hamiltonian_last_ = 0.0;
}

controller_interface::CallbackReturn BasicCartesianController::update_effort_commands()
{
  jacobian_pinv_ = jacobian_.completeOrthogonalDecomposition().pseudoInverse();
  jacobianT_pinv_ = jacobian_.transpose().colPivHouseholderQr().inverse();

  if (has_effort_states_)
  {
    estimated_wrench_.noalias() = jacobianT_pinv_ * (robot_efforts_ - effort_commands_);
  }

  pinocchio::getFrameJacobianTimeVariation(
    robot_model_, *robot_data_.get(), end_effector_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
    jacobian_dt_);

  // Compute `interaction_link` task space acceleration
  actual_accel_.noalias() = jacobian_ * robot_ddq_ + jacobian_dt_ * robot_dq_;
  accel_deviation_.noalias() = actual_accel_ - desired_pose_accel_;

  jsim_jpinv_ = robot_data_->M * jacobian_pinv_;
  actual_inertia_ = jacobianT_pinv_ * jsim_jpinv_;

  impedance_wrench_.noalias() =
    (desired_stiffness_ * pose_deviation_ + desired_damping_ * twist_deviation_) * (-1);

  desired_inertia_ = actual_inertia_;  // to compute Hamiltonian function

  accel_feedforward_ = jsim_jpinv_ * desired_pose_accel_;
  tau_desired_.noalias() =
    jacobian_.transpose() * impedance_wrench_ + accel_feedforward_ + robot_data_->g;

  effort_commands_ = cmd_lpf_alpha_ * tau_desired_ + (1.0 - cmd_lpf_alpha_) * effort_commands_;

  compute_hamiltonian();
  return controller_interface::CallbackReturn::SUCCESS;
}

void BasicCartesianController::publish_status()
{
  // Robot Hamiltonian - Impedance Hamiltonian
  status_msg_.data[0] = robot_data_->mechanical_energy - hamiltonian_filtered_;
  // Commands input power
  status_msg_.data[1] = robot_dq_.transpose() * effort_commands_;
  // Interaction power using estimated interaction wrench
  status_msg_.data[2] = actual_twist_.transpose() * estimated_wrench_;
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

  status_msg_.data[18] = accel_deviation_(0);
  status_msg_.data[19] = accel_deviation_(1);
  status_msg_.data[20] = accel_deviation_(2);
  status_msg_.data[21] = accel_deviation_(3);
  status_msg_.data[22] = accel_deviation_(4);
  status_msg_.data[23] = accel_deviation_(5);

  status_rt_publisher_->try_publish(status_msg_);
}

void BasicCartesianController::compute_hamiltonian()
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

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::BasicCartesianController, controller_interface::ControllerInterface)
