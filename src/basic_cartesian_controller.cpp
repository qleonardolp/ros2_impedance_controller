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
  status_msg_.data.resize(18, 0.0);
  // Initialize dynamic Eigen members
  tau_desired_.resize(get_dof());
  accel_feedforward_.resize(get_dof());
  jacobian_pinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
  jacobianT_pinv_ = Eigen::MatrixXd::Zero(kCartesianDim, get_dof());
  jsim_jpinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
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

  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
    robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();
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
  status_msg_.data[4] = hamiltonian_derivative_;
  // Impedance I/O power
  status_msg_.data[5] = twist_deviation_.transpose() * impedance_wrench_;

  actual_pose_.head<3>() = robot_data_.get()->oMf[end_effector_frame_].translation();
  status_msg_.data[6] = actual_pose_(0);
  status_msg_.data[7] = actual_pose_(1);
  status_msg_.data[8] = actual_pose_(2);

  status_rt_publisher_->try_publish(status_msg_);
}

void BasicCartesianController::zspace_diagnostics()
{
  /*
    if (status_rt_publisher_->trylock())
    {
      status_rt_publisher_->msg_.pose.position.x = pose_deviation_(0);
      status_rt_publisher_->msg_.pose.position.y = pose_deviation_(1);
      status_rt_publisher_->msg_.pose.position.z = pose_deviation_(2);
      // Using the quaternion vector as the angles
      status_rt_publisher_->msg_.pose.orientation.x = pose_deviation_(3);
      status_rt_publisher_->msg_.pose.orientation.y = pose_deviation_(4);
      status_rt_publisher_->msg_.pose.orientation.z = pose_deviation_(5);

      status_rt_publisher_->msg_.pose_twist.linear.x = twist_deviation_(0);
      status_rt_publisher_->msg_.pose_twist.linear.y = twist_deviation_(1);
      status_rt_publisher_->msg_.pose_twist.linear.z = twist_deviation_(2);
      status_rt_publisher_->msg_.pose_twist.angular.x = twist_deviation_(3);
      status_rt_publisher_->msg_.pose_twist.angular.y = twist_deviation_(4);
      status_rt_publisher_->msg_.pose_twist.angular.z = twist_deviation_(5);

      status_rt_publisher_->msg_.pose_accel.linear.x = estimated_wrench_(0);
      status_rt_publisher_->msg_.pose_accel.linear.y = estimated_wrench_(1);
      status_rt_publisher_->msg_.pose_accel.linear.z = estimated_wrench_(2);
      status_rt_publisher_->msg_.pose_accel.angular.x = estimated_wrench_(3);
      status_rt_publisher_->msg_.pose_accel.angular.y = estimated_wrench_(4);
      status_rt_publisher_->msg_.pose_accel.angular.z = estimated_wrench_(5);

      status_rt_publisher_->unlockAndPublish();
    }
  */
}

void BasicCartesianController::phase_space_diagnostics()
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
  //   desired_damping_.diagonal().segment(0, get_dof()));

  // Second term:
  status_msg_.data[13] = -(desired_damping_ * jacobian_ * robot_data_->Minv).trace();
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
