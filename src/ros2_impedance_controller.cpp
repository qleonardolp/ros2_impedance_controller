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

#include "ros2_impedance_controller/ros2_impedance_controller.hpp"

namespace ros2_impedance_controller
{
ImpedanceController::ImpedanceController()
: controller_interface::ControllerInterface(),
  rt_reference_ptr_(nullptr),
  reference_subscriber_(nullptr)
{
}

controller_interface::InterfaceConfiguration ImpedanceController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_config;
  command_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.joints)
  {
    command_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return command_config;
}

controller_interface::InterfaceConfiguration ImpedanceController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_config;
  state_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.joints)
  {
    state_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    state_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    state_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return state_config;
}

controller_interface::CallbackReturn ImpedanceController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  parameters_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");

  if (!configure_robot_model())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  auto qos_lowlatency = rclcpp::QoS(1);
  qos_lowlatency.best_effort().durability_volatile();
  qos_lowlatency.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

  zspace_publisher_ = get_node()->create_publisher<ReferenceType>("~/zspace", qos_lowlatency);
  realtime_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<ReferenceType>>(zspace_publisher_);

  if (params_.visualize_reference)
  {
    configure_visualization_marker();
    reference_marker_publisher_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
      "~/reference_marker", rclcpp::SystemDefaultsQoS());
  }

  reference_subscriber_ = get_node()->create_subscription<ReferenceType>(
    "~/reference", qos_lowlatency,
    [this](const ReferenceType::SharedPtr msg)
    {
      static uint16_t downsample = 0;
      rt_reference_ptr_.writeFromNonRT(msg);
      if (params_.visualize_reference)
      {
        ++downsample;
        if (0 == downsample % 5)
        {
          marker_.pose = msg.get()->pose;
          reference_marker_publisher_->publish(marker_);
          downsample = 0;
        }
      }
    });

  sensor_wrench_.setZero();
  interaction_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
    params_.ft_sensor_topic, qos_lowlatency,
    [this](const geometry_msgs::msg::Wrench::SharedPtr wrench)
    {
      // TODO(@me): compensate the sensor weight
      Vector6d sensor_wrench_raw;
      sensor_wrench_raw.head<3>() =
        Eigen::Vector3d(wrench->force.x, wrench->force.y, wrench->force.z);
      sensor_wrench_raw.tail<3>() =
        Eigen::Vector3d(wrench->torque.x, wrench->torque.y, wrench->torque.z);
      const double lpf_alpha = 0.556862724;  // Low-pass filter, 200 Hz cutoff frequency
      sensor_wrench_ = lpf_alpha * sensor_wrench_raw + (1 - lpf_alpha) * sensor_wrench_;
    });

  // Initialize dynamic Eigen members
  robot_positions_.resize(degrees_of_freedom_);
  robot_velocities_.resize(degrees_of_freedom_);
  robot_velocities_last_.resize(degrees_of_freedom_);
  robot_accelerations_.resize(degrees_of_freedom_);
  robot_efforts_.resize(degrees_of_freedom_);
  effort_commands_.resize(degrees_of_freedom_);
  twist_compensation_.resize(degrees_of_freedom_);
  accel_feedforward_.resize(degrees_of_freedom_);
  impedance_torques_.resize(degrees_of_freedom_);
  jacobian_ = Matrix6Xd::Zero(kCartesianSpaceDim, degrees_of_freedom_);
  jacobian_derivative_ = Matrix6Xd::Zero(kCartesianSpaceDim, degrees_of_freedom_);
  jacobian_pinv_ = Eigen::MatrixXd::Zero(degrees_of_freedom_, kCartesianSpaceDim);
  jsim_jpinv_ = Eigen::MatrixXd::Zero(degrees_of_freedom_, kCartesianSpaceDim);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  parameters_client_.reset();
  rt_reference_ptr_.reset();
  reference_subscriber_.reset();
  interaction_subscriber_.reset();
  reference_marker_publisher_.reset();
  realtime_publisher_.reset();
  zspace_publisher_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  impedance_ioenergy_ = 0;
  impedance_power_last_ = 0;

  // Dynamic size members (joint space dim)
  jacobian_.setZero();
  jacobian_derivative_.setZero();
  effort_commands_.setZero();
  twist_compensation_.setZero();
  accel_feedforward_.setZero();
  impedance_torques_.setZero();
  robot_velocities_last_.setZero();

  // ros2_control interfaces
  effort_command_interfaces_.resize(degrees_of_freedom_, nullptr);
  position_interfaces_.resize(degrees_of_freedom_, nullptr);
  velocity_interfaces_.resize(degrees_of_freedom_, nullptr);
  effort_interfaces_.resize(degrees_of_freedom_, nullptr);

  // Constant size members (task space dim)
  pose_deviation_.setZero();
  twist_deviation_.setZero();
  desired_pose_accel_.setZero();
  impedance_wrench_.setZero();

  desired_quaternion_.setIdentity();
  desired_position_.setZero();
  desired_twist_.setZero();
  actual_twist_.setZero();
  actual_accel_.setZero();

  for (const auto & interface : state_interfaces_)  // LoanedStateInterface from the base class
  {
    const std::string & joint_name = interface.get_prefix_name();
    const std::string & interface_type = interface.get_interface_name();

    auto it = std::find(params_.joints.begin(), params_.joints.end(), joint_name);
    if (it == params_.joints.end()) continue;
    const size_t index = std::distance(params_.joints.begin(), it);

    if (interface_type == hardware_interface::HW_IF_POSITION)
      position_interfaces_[index] = &interface;
    else if (interface_type == hardware_interface::HW_IF_VELOCITY)
      velocity_interfaces_[index] = &interface;
    else if (interface_type == hardware_interface::HW_IF_EFFORT)
      effort_interfaces_[index] = &interface;
  }

  for (auto & interface : command_interfaces_)  // LoanedCommandInterface from the base class
  {
    if (interface.get_interface_name() != hardware_interface::HW_IF_EFFORT) continue;
    const std::string & joint_name = interface.get_prefix_name();
    auto it = std::find(params_.joints.begin(), params_.joints.end(), joint_name);
    if (it == params_.joints.end()) continue;
    const size_t index = std::distance(params_.joints.begin(), it);
    effort_command_interfaces_[index] = &interface;
  }

  for (size_t i = 0; i < degrees_of_freedom_; i++)
  {
    if (!effort_command_interfaces_[i])
    {
      RCLCPP_ERROR(logger, "Command interface for joint %zu is null!", i);
      return CallbackReturn::ERROR;
    }
  }
  // Reset reference buffer
  rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);

  RCLCPP_WARN(logger, "Activated successfully!");

  clock_time_last_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  effort_command_interfaces_.clear();
  position_interfaces_.clear();
  velocity_interfaces_.clear();
  effort_interfaces_.clear();

  rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  ellapsed_time_ = static_cast<double>((time - clock_time_last_).nanoseconds()) * 1E-9;
  // Read state interfaces and update robot
  if (!update_robot())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to update robot model with state data");
    return controller_interface::return_type::ERROR;
  }

  pinocchio::computeFrameJacobian(
    robot_model_, *robot_data_.get(), robot_positions_, end_effector_frame_,
    pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

  update_deviations();  // needs jacobian_ updated

  pinocchio::getFrameJacobianTimeVariation(
    robot_model_, *robot_data_.get(), end_effector_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
    jacobian_derivative_);

  jacobian_pinv_ = jacobian_.completeOrthogonalDecomposition().pseudoInverse();

  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
    robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();
  jsim_jpinv_ = robot_data_->M * jacobian_pinv_;

  twist_compensation_.noalias() =
    (robot_data_->C - jsim_jpinv_ * jacobian_derivative_) * robot_velocities_;

  if (!desired_pose_accel_.isZero())
  {
    accel_feedforward_ = jsim_jpinv_ * desired_pose_accel_;
  }

  impedance_wrench_.noalias() =
    desired_stiffness_ * pose_deviation_ + desired_damping_ * twist_deviation_;

  if (inertia_shaping_)
  {
    impedance_torques_.noalias() =
      jsim_jpinv_ * desired_inertia_inv_ * (impedance_wrench_ + sensor_wrench_);
    impedance_torques_.noalias() -= jacobian_.transpose() * sensor_wrench_;
  }
  else
  {
    impedance_torques_.noalias() = jacobian_.transpose() * impedance_wrench_;
  }

  effort_commands_ = accel_feedforward_ + impedance_torques_ + twist_compensation_ + robot_data_->g;

  for (uint8_t k = 0; k < degrees_of_freedom_; ++k)
  {
    if (!effort_command_interfaces_[k]->set_value(effort_commands_(k)))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }
  }

  if (!inertia_shaping_)
  {
    // Operational (task) space inertia matrix
    desired_inertia_.noalias() =
      jacobian_.transpose().completeOrthogonalDecomposition().pseudoInverse() * jsim_jpinv_;
  }
  compute_inout_energy();  // needs desired_inertia_ updated
  compute_hamiltonian();

  static uint8_t sample_diagnostics = 0;
  if (0 == sample_diagnostics % 4)  // downsampling the publisher 4x
  {
    // publish_impedance_space();
    ph_diagnostics();
    sample_diagnostics = 0;
  }
  ++sample_diagnostics;

  clock_time_last_ = time;
  return controller_interface::return_type::OK;
}

bool ImpedanceController::configure_robot_model()
{
  parameters_client_->wait_for_service();
  auto parameters_future = parameters_client_->get_parameters(
    {"robot_description"},
    std::bind(&ImpedanceController::robot_description_param_cb, this, std::placeholders::_1));

  parameters_future.wait();

  while (robot_urdf_.empty())
  {
  }

  pinocchio::urdf::buildModelFromXML(robot_urdf_, robot_model_);
  robot_data_ = std::make_shared<pinocchio::Data>(robot_model_);

  if (!robot_model_.existFrame(params_.interaction_link))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Frame '%s' not found!", params_.interaction_link.c_str());
    return false;
  }
  end_effector_frame_ = robot_model_.getFrameId(params_.interaction_link);
  robot_model_.gravity =
    pinocchio::Motion(Eigen::Vector3d(0.0, 0.0, -9.78265), Eigen::Vector3d::Zero());
  RCLCPP_INFO(
    get_node()->get_logger(), "Robot model %s loaded with %d DOFs", robot_model_.name.c_str(),
    robot_model_.nq);
  return true;
}

bool ImpedanceController::update_robot()
{
  for (uint8_t k = 0; k < degrees_of_freedom_; k++)
  {
    std::optional position = position_interfaces_[k]->get_optional();
    std::optional velocity = velocity_interfaces_[k]->get_optional();
    std::optional effort = effort_interfaces_[k]->get_optional();

    if (!position.has_value() || !velocity.has_value() || !effort.has_value())
    {
      return false;
    }

    robot_positions_(k) = position.value();
    robot_velocities_(k) = velocity.value();
    robot_efforts_(k) = effort.value();

    // Finite difference
    robot_accelerations_(k) = (robot_velocities_(k) - robot_velocities_last_(k)) / ellapsed_time_;
    robot_velocities_last_(k) = robot_velocities_(k);
  }
  pinocchio::computeAllTerms(robot_model_, *robot_data_.get(), robot_positions_, robot_velocities_);
  return true;
}

void ImpedanceController::update_deviations()
{
  actual_twist_.noalias() = jacobian_ * robot_velocities_;

  if (*rt_reference_ptr_.readFromNonRT() != nullptr)
  {
    desired_kpose_ = *rt_reference_ptr_.readFromNonRT()->get();

    desired_position_.x() = desired_kpose_.pose.position.x;
    desired_position_.y() = desired_kpose_.pose.position.y;
    desired_position_.z() = desired_kpose_.pose.position.z;

    desired_quaternion_.w() = desired_kpose_.pose.orientation.w;
    desired_quaternion_.x() = desired_kpose_.pose.orientation.x;
    desired_quaternion_.y() = desired_kpose_.pose.orientation.y;
    desired_quaternion_.z() = desired_kpose_.pose.orientation.z;
    desired_quaternion_.normalize();

    pose_deviation_.head<3>().noalias() =
      desired_position_ - robot_data_.get()->oMf[end_effector_frame_].translation();
    pose_deviation_.tail<3>().noalias() = pinocchio::log3(
      desired_quaternion_.toRotationMatrix() *
      robot_data_.get()->oMf[end_effector_frame_].rotation().transpose());

    desired_twist_(0) = desired_kpose_.pose_twist.linear.x;
    desired_twist_(1) = desired_kpose_.pose_twist.linear.y;
    desired_twist_(2) = desired_kpose_.pose_twist.linear.z;
    desired_twist_(3) = desired_kpose_.pose_twist.angular.x;
    desired_twist_(4) = desired_kpose_.pose_twist.angular.y;
    desired_twist_(5) = desired_kpose_.pose_twist.angular.z;

    twist_deviation_.noalias() = desired_twist_ - actual_twist_;

    desired_pose_accel_(0) = desired_kpose_.pose_accel.linear.x;
    desired_pose_accel_(1) = desired_kpose_.pose_accel.linear.y;
    desired_pose_accel_(2) = desired_kpose_.pose_accel.linear.z;
    desired_pose_accel_(3) = desired_kpose_.pose_accel.angular.x;
    desired_pose_accel_(4) = desired_kpose_.pose_accel.angular.y;
    desired_pose_accel_(5) = desired_kpose_.pose_accel.angular.z;
  }
  else
  {
    pose_deviation_.setZero();
    twist_deviation_.noalias() = -actual_twist_;
    desired_pose_accel_.setZero();
  }
}

void ImpedanceController::publish_impedance_space()
{
  diagnostics_msg_.pose.position.x = pose_deviation_(0);
  diagnostics_msg_.pose.position.y = pose_deviation_(1);
  diagnostics_msg_.pose.position.z = pose_deviation_(2);
  // Using the quaternion vector as the angles
  diagnostics_msg_.pose.orientation.x = pose_deviation_(3);
  diagnostics_msg_.pose.orientation.y = pose_deviation_(4);
  diagnostics_msg_.pose.orientation.z = pose_deviation_(5);

  diagnostics_msg_.pose_twist.linear.x = twist_deviation_(0);
  diagnostics_msg_.pose_twist.linear.y = twist_deviation_(1);
  diagnostics_msg_.pose_twist.linear.z = twist_deviation_(2);
  diagnostics_msg_.pose_twist.angular.x = twist_deviation_(3);
  diagnostics_msg_.pose_twist.angular.y = twist_deviation_(4);
  diagnostics_msg_.pose_twist.angular.z = twist_deviation_(5);

  diagnostics_msg_.pose_accel.linear.x = sensor_wrench_(0);
  diagnostics_msg_.pose_accel.linear.y = sensor_wrench_(1);
  diagnostics_msg_.pose_accel.linear.z = sensor_wrench_(2);
  diagnostics_msg_.pose_accel.angular.x = sensor_wrench_(3);
  diagnostics_msg_.pose_accel.angular.y = sensor_wrench_(4);
  diagnostics_msg_.pose_accel.angular.z = sensor_wrench_(5);

  realtime_publisher_->try_publish(diagnostics_msg_);
}

void ImpedanceController::ph_diagnostics()
{
  static bool is_passive = true;
  is_passive = impedance_ioenergy_ > impedance_hamiltonian_;

  // JS control power
  diagnostics_msg_.pose_twist.linear.x = robot_velocities_.transpose() * effort_commands_;
  // JS total power
  diagnostics_msg_.pose_twist.linear.y = robot_velocities_.transpose() * robot_efforts_;
  // Interaction power
  diagnostics_msg_.pose_twist.linear.z = actual_twist_.transpose() * sensor_wrench_;
  // Impedance Hamiltonian
  diagnostics_msg_.pose_twist.angular.x = impedance_hamiltonian_;
  // Impedance Hamiltonian I/O energy
  diagnostics_msg_.pose_twist.angular.y = impedance_ioenergy_;
  // N-DoF Passivity
  diagnostics_msg_.pose_twist.angular.z = is_passive;

  realtime_publisher_->try_publish(diagnostics_msg_);
}

void ImpedanceController::compute_inout_energy()
{
  static double power = 0;  // [y.u]_z
  power = twist_deviation_.transpose() * (sensor_wrench_ - desired_inertia_ * desired_pose_accel_);
  // Trapezoidal integration
  impedance_ioenergy_ += (power + impedance_power_last_) * ellapsed_time_ * 0.5;
  impedance_power_last_ = power;
}

void ImpedanceController::compute_hamiltonian()
{
  impedance_hamiltonian_ = twist_deviation_.transpose() * desired_inertia_ * twist_deviation_;
  impedance_hamiltonian_ += pose_deviation_.transpose() * desired_stiffness_ * pose_deviation_;
  impedance_hamiltonian_ = 0.5 * impedance_hamiltonian_;
}

void ImpedanceController::robot_description_param_cb(
  std::shared_future<std::vector<rclcpp::Parameter>> future)
{
  robot_urdf_ = future.get().at(0).as_string();
}

controller_interface::CallbackReturn ImpedanceController::read_parameters()
{
  params_ = param_listener_->get_params();
  degrees_of_freedom_ = params_.degrees_of_freedom;

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  desired_stiffness_ = Vector6d(params_.stiffness.data()).asDiagonal();
  Vector6d inertia_diagonal = kDefaultInertia;

  if (std::fpclassify(params_.taskspace_mass) == FP_ZERO)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Desired Cartesian mass is 0.0. Inertia shaping disabled.");

    if (!params_.damping.empty())
    {
      desired_damping_.diagonal() = Vector6d(params_.damping.data());
    }
    else
    {
      desired_damping_.diagonal() =
        2 * kDampingRatio *
        (inertia_diagonal.array() * desired_stiffness_.diagonal().array()).abs().sqrt();
    }
    inertia_shaping_ = false;
  }
  else
  {
    const double mass = params_.taskspace_mass;
    const double I = 0.4 * mass * (0.425 * 0.425);  // sphere moment of inertia
    inertia_diagonal << mass, mass, mass, I, I, I;
    desired_inertia_.diagonal() = inertia_diagonal;
    desired_inertia_inv_ = inertia_diagonal.cwiseInverse().asDiagonal();
    // Critically damped: D = 2 * sqrt(K * M)
    desired_damping_.diagonal() =
      2 * (inertia_diagonal.array() * desired_stiffness_.diagonal().array()).abs().sqrt();
    inertia_shaping_ = true;
  }

  RCLCPP_INFO_STREAM(
    get_node()->get_logger(), "Damping matrix diagonal: \n"
                                << desired_damping_.diagonal());
  return controller_interface::CallbackReturn::SUCCESS;
}

void ImpedanceController::configure_visualization_marker()
{
  marker_ = visualization_msgs::msg::Marker();
  marker_.header.frame_id = "world";
  marker_.ns = "impedance_controller/reference";
  marker_.id = 23;  // Random ID
  marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_.action = visualization_msgs::msg::Marker::MODIFY;
  marker_.scale.x = 0.006;
  marker_.color.r = static_cast<float>(0.99);
  marker_.color.b = static_cast<float>(0.99);
  marker_.color.a = static_cast<float>(0.80);

  geometry_msgs::msg::Point origin_point;  // constructor assign zeros
  geometry_msgs::msg::Point x_point;
  geometry_msgs::msg::Point y_point;
  geometry_msgs::msg::Point z_point;
  x_point.x = 0.1;
  y_point.y = 0.1;
  z_point.z = 0.1;

  marker_.points.push_back(origin_point);
  marker_.points.push_back(x_point);
  marker_.points.push_back(origin_point);
  marker_.points.push_back(y_point);
  marker_.points.push_back(origin_point);
  marker_.points.push_back(z_point);
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::ImpedanceController, controller_interface::ControllerInterface)
