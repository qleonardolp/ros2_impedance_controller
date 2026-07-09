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

  zspace_window_ = params_.zspace_window;
  zspace_points_ = std::make_shared<SlidingWindow>(zspace_window_, 3);
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

  zspace_normal_last_ << 1.0, 0.0, 0.0;
  zspace_normal_.setOnes();
  zspace_counter_ = 0;

  // PH related
  impedance_expected_input_.setZero();
  hamiltonian_filtered_ = 0.0;
  hamiltonian_last_ = 0.0;
}

controller_interface::CallbackReturn BasicCartesianController::update_effort_commands()
{
  update_start_ = steady_clock_->now();

  compute_osim();
  desired_inertia_ = actual_inertia_;  // to compute Hamiltonian function

  rls_identification();
  zsapce_identification();

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
  status_msg_.data[4] = actual_inertia_(2, 2);  // trying to visualize m_zz
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

  status_msg_.data[18] = zspace_normal_last_(0);
  status_msg_.data[19] = zspace_normal_last_(1);
  status_msg_.data[20] = zspace_normal_last_(2);
  status_msg_.data[21] = zspace_is_steady_;
  status_msg_.data[22] = desired_damping_.diagonal()(2) / rls_theta_(0);
  status_msg_.data[23] = desired_stiffness_.diagonal()(2) / rls_theta_(1);

  status_msg_.data[24] = (update_end_ - update_start_).seconds();
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

int BasicCartesianController::zsapce_identification()
{
  // Check data integrity
  if (pose_deviation_.hasNaN() || twist_.hasNaN() || accel_.hasNaN()) return 1;

  zspace_new_ << pose_deviation_(2), twist_(2), accel_(2);
  // Check steady state
  zspace_is_steady_ = zspace_new_.head<2>().norm() < 1e-3;  // accel is noisy so we neglect it
  if (zspace_is_steady_) return 2;

  zspace_points_->push(zspace_new_);

  zspace_counter_++;
  if (zspace_counter_ % zspace_window_ == 0)
  {
    zspace_mean_ = zspace_points_->get_buffer().colwise().mean();  // window mean
    for (size_t i = 0; i < zspace_window_; i++)
    {
      // Reusing `zspace_new_` as the centered point auxiliary variable
      zspace_new_.noalias() = zspace_points_->get_buffer().row(i);
      zspace_new_ -= zspace_mean_;
      zspace_M_ += zspace_new_ * zspace_new_.transpose();
    }
    zspace_M_ = zspace_M_ / static_cast<double>(zspace_window_);
    zspace_Meig_.compute(zspace_M_);
    // Eigenvector corresponding to the smallest eigenvalue:
    zspace_normal_ = zspace_Meig_.eigenvectors().col(0);
    // Constrain negative params (k,d,m):
    zspace_normal_last_ = (zspace_normal_(0) > 0) ? zspace_normal_ : -zspace_normal_;
    zspace_counter_ = 0;
  }
  return 0;
}

void BasicCartesianController::compute_osim()
{
  pinocchio::cholesky::decompose(robot_model_, *robot_data_.get());
  pinocchio::cholesky::computeMinv(robot_model_, *robot_data_.get());

  actual_inertia_ = (jacobian_ * robot_data_->Minv * jacobian_.transpose())
                      .completeOrthogonalDecomposition()
                      .pseudoInverse();
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::BasicCartesianController, controller_interface::ControllerInterface)
