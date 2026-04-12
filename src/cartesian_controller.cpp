// Copyright (c) 2025, qleonardolp
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

#include "ros2_impedance_controller/cartesian_controller.hpp"

namespace ros2_impedance_controller
{
CartesianController::CartesianController() : ImpedanceControllerBase() {}

void CartesianController::declare_parameters()
{
  param_listener_ = std::make_shared<::cartesian_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn CartesianController::read_parameters()
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

  if (std::fpclassify(params_.taskspace_mass) == FP_ZERO)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Desired Cartesian mass is 0.0. Inertia shaping disabled.");

    if (params_.damping.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'damping' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    desired_damping_ = Vector6d(params_.damping.data()).asDiagonal();
    desired_inertia_.setIdentity();
    inertia_shaping_ = false;
  }
  else
  {
    const double mass = params_.taskspace_mass;
    const double I = 0.4 * mass * (0.425 * 0.425);  // sphere moment of inertia
    desired_inertia_.diagonal() << mass, mass, mass, I, I, I;
    desired_inertia_inv_ = desired_inertia_.diagonal().cwiseInverse().asDiagonal();
    // Damping: D = 2 * ζ * sqrt(K * M)
    desired_damping_.diagonal() =
      2 * kDampingRatio *
      (desired_inertia_.diagonal().array() * desired_stiffness_.diagonal().array()).abs().sqrt();
    inertia_shaping_ = true;
  }

  RCLCPP_INFO_STREAM(
    get_node()->get_logger(),
    "Damping matrix diagonal: " << desired_damping_.diagonal().transpose());
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianController::custom_configuration()
{
  auto qos_lowlatency = rclcpp::QoS(1);
  qos_lowlatency.best_effort().durability_volatile();
  qos_lowlatency.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

  sensor_wrench_.setZero();
  if (!params_.ft_sensor_topic.empty())
  {
    int_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
      params_.ft_sensor_topic, qos_lowlatency,
      [this](const geometry_msgs::msg::Wrench::SharedPtr wrench)
      {
        Vector6d sensor_wrench_raw;
        sensor_wrench_raw.head<3>() =
          Eigen::Vector3d(wrench->force.x, wrench->force.y, wrench->force.z);
        sensor_wrench_raw.tail<3>() =
          Eigen::Vector3d(wrench->torque.x, wrench->torque.y, wrench->torque.z);
        sensor_wrench_ = cmd_lpf_alpha_ * sensor_wrench_raw + (1 - cmd_lpf_alpha_) * sensor_wrench_;
      });
  }

  status_msg_.data.resize(24, 0.0);
  // Initialize dynamic Eigen members
  tau_desired_.resize(get_dof());
  twist_compensation_.resize(get_dof());
  accel_feedforward_.resize(get_dof());
  impedance_torques_.resize(get_dof());
  jacobian_pinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
  jacobianT_pinv_ = Eigen::MatrixXd::Zero(kCartesianDim, get_dof());
  jsim_jpinv_ = Eigen::MatrixXd::Zero(get_dof(), kCartesianDim);
  jsim_jpinv_dj_ = Eigen::MatrixXd::Zero(get_dof(), get_dof());
  jacobian_dt_ = Eigen::MatrixXd::Zero(kCartesianDim, get_dof());
}

void CartesianController::custom_activation()
{
  // Dynamic size members (joint space dim)
  jacobian_dt_.setZero();
  twist_compensation_.setZero();
  accel_feedforward_.setZero();
  impedance_torques_.setZero();
  tau_desired_.setZero();

  impedance_wrench_.setZero();
  estimated_wrench_.setZero();

  // PH related
  impedance_expected_input_.setZero();
  hamiltonian_filtered_ = 0.0;
  hamiltonian_last_ = 0.0;

  // Sys ID
  ls_input_.setOnes();
  ls_out_.setZero();
}

controller_interface::CallbackReturn CartesianController::update_effort_commands()
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
  jsim_jpinv_dj_ = jsim_jpinv_ * jacobian_dt_;

  actual_inertia_ = jacobianT_pinv_ * jsim_jpinv_;

  impedance_wrench_.noalias() =
    (desired_stiffness_ * pose_deviation_ + desired_damping_ * twist_deviation_) * (-1);

  if (inertia_shaping_)
  {
    impedance_torques_.noalias() =
      jsim_jpinv_ * desired_inertia_inv_ * (impedance_wrench_ + sensor_wrench_);
    impedance_torques_.noalias() -= jacobian_.transpose() * sensor_wrench_;
  }
  else
  {
    impedance_torques_.noalias() = jacobian_.transpose() * impedance_wrench_;
    desired_inertia_ = actual_inertia_;  // to compute Hamiltonian function
  }

  accel_feedforward_ = jsim_jpinv_ * desired_pose_accel_;
  twist_compensation_.noalias() = (robot_data_->C - jsim_jpinv_dj_) * robot_dq_;
  tau_desired_ = accel_feedforward_ + impedance_torques_ + twist_compensation_ + robot_data_->g;

  effort_commands_ = cmd_lpf_alpha_ * tau_desired_ + (1.0 - cmd_lpf_alpha_) * effort_commands_;

  compute_hamiltonian();
  zspace_regression();
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianController::publish_status()
{
  // Robot Hamiltonian - Impedance Hamiltonian
  status_msg_.data[0] = robot_data_->mechanical_energy - hamiltonian_filtered_;
  // Commands input power
  status_msg_.data[1] = robot_dq_.transpose() * effort_commands_;
  // Interaction power using F/T Sensor (ground truth)
  status_msg_.data[2] = actual_twist_.transpose() * sensor_wrench_;
  // Impedance Hamiltonian
  status_msg_.data[3] = hamiltonian_filtered_;
  // Impedance Hamiltonian derivative
  status_msg_.data[4] = hamiltonian_derivative_;
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

void CartesianController::phase_space_diagnostics()
{
  status_msg_.data[0] = pose_deviation_(0);
  status_msg_.data[1] = pose_deviation_(1);
  status_msg_.data[2] = pose_deviation_(2);
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
  status_msg_.data[12] = -(jsim_jpinv_dj_ * robot_data_->Minv).trace();

  // TODO(@qleonardolp) investigate the trace for non square matrices (nDoF < m)
  // Eigen::DiagonalMatrix<double, Eigen::Dynamic, kCartesianDim> damping(
  //   desired_damping_.diagonal().segment(0, get_dof()));

  // Second term:
  if (inertia_shaping_)
  {
    status_msg_.data[13] =
      -(actual_inertia_ * desired_inertia_inv_ * desired_damping_ * jacobian_ * robot_data_->Minv)
         .trace();
  }
  else
  {
    status_msg_.data[13] = -(desired_damping_ * jacobian_ * robot_data_->Minv).trace();
  }
}

void CartesianController::compute_hamiltonian()
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

void CartesianController::zspace_regression()
{
  // Roll u and y
  for (size_t k = 3 - 1; k > 0; --k)
  {
    ls_input_.col(k) = ls_input_.col(k - 1);
    ls_out_.col(k) = ls_out_.col(k - 1);
  }
  // Update
  ls_input_.col(0).head<kCartesianDim>().noalias() =
    desired_pose_accel_ + desired_inertia_inv_ * (impedance_wrench_ + sensor_wrench_);
  // ls_input_.col(0) last element is 1
  ls_out_.col(0).noalias() = actual_accel_;

  // Compute S_{\Lambda}
  ls_S_lambda_.noalias() =
    ls_out_ * ls_input_.transpose() * (ls_input_ * ls_input_.transpose()).inverse();

  ls_b_.noalias() = ls_S_lambda_.rightCols<1>();  // S_lambda_ last column

  // TODO(@me): ls_b_ is near zero, but requires LPF
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::CartesianController, controller_interface::ControllerInterface)
