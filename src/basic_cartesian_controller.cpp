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
  steady_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  status_msg_.data.resize(25, 0.0);

  // Initialize dynamic Eigen members
  tau_desired_.resize(get_dof());

  zspace_id_ = std::make_shared<ZSpaceIdentification>(params_.zspace_window);
}

void BasicCartesianController::custom_activation()
{
  // Dynamic size members (joint space dim)
  tau_desired_.setZero();

  // Fixed size members

  // RLS initialization
  rls_theta_(0) = desired_damping_.diagonal()(2);    // d/m
  rls_theta_(1) = desired_stiffness_.diagonal()(2);  // k/m
  rls_cov_ = 10 * Eigen::Matrix2d::Identity();

  zspace_id_->reset_estimation();
}

controller_interface::CallbackReturn BasicCartesianController::update_effort_commands()
{
  update_start_ = steady_clock_->now();

  // rls_identification();
  zspace_ret_ = zspace_id_->update(pose_deviation_, twist_deviation_, accel_, 0);  // x-axis

  impedance_wrench_.noalias() =
    desired_stiffness_ * pose_deviation_ + desired_damping_ * twist_deviation_;

  tau_desired_.noalias() = -jacobian_.transpose() * impedance_wrench_;

  if (params_.gravity_compensation)
  {
    tau_desired_ += robot_data_->g;
  }

  effort_commands_ = kCmdAlpha * tau_desired_ + (1.0 - kCmdAlpha) * effort_commands_;

  compute_osim();
  desired_inertia_ = actual_inertia_;  // to compute Hamiltonian function
  compute_hamiltonian();

  update_end_ = steady_clock_->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

void BasicCartesianController::publish_status()
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
  status_msg_.data[4] = actual_inertia_(0, 0);
  // Impedance I/O power
  status_msg_.data[5] = -twist_deviation_.transpose() * impedance_wrench_;

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

  status_msg_.data[18] = accel_(0);
  status_msg_.data[19] = accel_(1);
  status_msg_.data[20] = accel_(2);
  status_msg_.data[21] = zspace_id_->get_normal()(0);
  status_msg_.data[22] = zspace_id_->get_normal()(1);
  status_msg_.data[23] = zspace_id_->get_normal()(2);

  status_msg_.data[24] = (update_end_ - update_start_).seconds();
  status_rt_publisher_->try_publish(status_msg_);
}

int BasicCartesianController::rls_identification()
{
  // Check data integrity
  if (pose_deviation_.hasNaN() || twist_.hasNaN() || accel_.hasNaN()) return 1;

  // Update phi (Running for 'z' axis only)
  rls_phi_(0) = -twist_(2);           // dx (*d/m)
  rls_phi_(1) = -pose_deviation_(2);  // x_d - x (*k/m)
  rls_y_ = accel_(2);

  // Check steady state
  rls_is_steady_ = rls_phi_.norm() < 1e-3;  // loosely speaking: 1 mm for zero velocity
  if (rls_is_steady_) return 2;

  // Update gain
  rls_gain_den_ = rls_lambda_ + rls_phi_.transpose() * rls_cov_ * rls_phi_;
  rls_gain_.noalias() = rls_cov_ * rls_phi_ * (1.0 / rls_gain_den_);
  // Update covariance
  rls_cov_ =
    (Eigen::Matrix2d::Identity() - rls_gain_ * rls_phi_.transpose()) * rls_cov_ / rls_lambda_;
  // Update params: tht_n = tht_{n-1} + K(n)*e(n)
  rls_error_ = rls_y_ - rls_phi_.transpose() * rls_theta_;
  rls_theta_ = rls_theta_ + rls_gain_ * rls_error_;
  rls_error2_ = rls_error_ * rls_error_;
  return 0;
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::BasicCartesianController, controller_interface::ControllerInterface)
