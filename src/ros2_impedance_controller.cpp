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
    declare_parameters();
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

  configure_visualization_marker();

  reference_marker_publisher_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
    "~/reference_marker", rclcpp::SystemDefaultsQoS());

  reference_subscriber_ = get_node()->create_subscription<ReferenceType>(
    "~/reference", rclcpp::ClockQoS(),
    [this](const ReferenceType::SharedPtr msg)
    {
      rt_reference_ptr_.writeFromNonRT(msg);
      marker_downsample_++;
      if (0 == marker_downsample_ % 5)
      {
        marker_.pose = msg.get()->pose;
        reference_marker_publisher_->publish(marker_);
        marker_downsample_ = 0;
      }
    });

  interaction_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
    "end_effector_ft_sensor", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::Wrench::SharedPtr wrench)
    {
      // TODO(@me): compensate the sensor weight
      // TODO(@me): low-pass filter the sensor data.
      interaction_wrench_(0) = wrench->force.x;
      interaction_wrench_(1) = wrench->force.y;
      interaction_wrench_(2) = wrench->force.z;
      interaction_wrench_(3) = wrench->torque.x;
      interaction_wrench_(4) = wrench->torque.y;
      interaction_wrench_(5) = wrench->torque.z;
    });

  // Initialize dynamic Eigen members
  robot_positions_ = VectorXd::Zero(degrees_of_freedom_);
  robot_velocities_ = VectorXd::Zero(degrees_of_freedom_);
  robot_efforts_ = VectorXd::Zero(degrees_of_freedom_);
  effort_commands_ = VectorXd::Zero(degrees_of_freedom_);
  gravity_compensation_ = VectorXd::Zero(degrees_of_freedom_);
  twist_compensation_ = VectorXd::Zero(degrees_of_freedom_);
  jacobian_ = Matrix6Xd::Zero(6, degrees_of_freedom_);
  jacobian_derivative_ = Matrix6Xd::Zero(6, degrees_of_freedom_);
  jacobian_pinverse_ = Eigen::MatrixXd::Zero(degrees_of_freedom_, 6);
  coriolis_ = Eigen::MatrixXd::Zero(degrees_of_freedom_, degrees_of_freedom_);

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

  // Dynamic size members
  coriolis_.setZero();
  jacobian_.setZero();
  jacobian_derivative_.setZero();
  effort_commands_.setZero();
  gravity_compensation_.setZero();
  twist_compensation_.setZero();
  accel_feedforward_.setZero();

  // ros2_control interfaces
  effort_command_interfaces_.resize(degrees_of_freedom_, nullptr);
  position_interfaces_.resize(degrees_of_freedom_, nullptr);
  velocity_interfaces_.resize(degrees_of_freedom_, nullptr);
  effort_interfaces_.resize(degrees_of_freedom_, nullptr);

  // Constant size members
  pose_deviation_.setZero();
  twist_deviation_.setZero();
  desired_pose_accel_.setZero();
  interaction_wrench_.setZero();
  impedance_wrench_.setZero();
  inertia_ratio_ = Matrix6d::Identity();

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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
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

  jacobian_pinverse_ = jacobian_.completeOrthogonalDecomposition().pseudoInverse();

  // Computes the upper triangular part of the joint space inertia matrix M
  pinocchio::crba(robot_model_, *robot_data_.get(), robot_positions_);
  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
    robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  Eigen::MatrixXd m_prod_jinv = robot_data_->M * jacobian_pinverse_;

  coriolis_ = pinocchio::getCoriolisMatrix(robot_model_, *robot_data_.get());
  twist_compensation_ = (coriolis_ - m_prod_jinv * jacobian_derivative_) * robot_velocities_;

  accel_feedforward_ = m_prod_jinv * desired_pose_accel_;

  impedance_wrench_ =
    taskspace_stiffness_ * pose_deviation_ + taskspace_damping_ * twist_deviation_;

  if (inertia_shaping_)
  {
    // Joint space inertia matrix (JSIM):
    pinocchio::computeMinverse(robot_model_, *robot_data_.get(), robot_positions_);
    robot_data_->Minv.triangularView<Eigen::StrictlyLower>() =
      robot_data_->Minv.transpose().triangularView<Eigen::StrictlyLower>();
    // OSIM * OSIM_d^{-1}:
    Eigen::MatrixXd osim = (jacobian_ * robot_data_->Minv * jacobian_.transpose()).inverse();
    inertia_ratio_ = osim * taskspace_inertia_inv_;
    impedance_wrench_ = inertia_ratio_ * impedance_wrench_ +
                        (inertia_ratio_ - Matrix6d::Identity()) * interaction_wrench_;
  }

  gravity_compensation_ =
    pinocchio::computeGeneralizedGravity(robot_model_, *robot_data_.get(), robot_positions_);

  effort_commands_ = jacobian_.transpose() * impedance_wrench_ + accel_feedforward_ +
                     twist_compensation_ + gravity_compensation_;

  for (uint8_t k = 0; k < degrees_of_freedom_; ++k)
  {
    if (!effort_command_interfaces_[k]->set_value(effort_commands_(k)))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }
  }
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
  }
  return true;
}

void ImpedanceController::update_deviations()
{
  Vector6d actual_twist = jacobian_ * robot_velocities_;

  if (*rt_reference_ptr_.readFromNonRT() != nullptr)
  {
    ReferenceType desired_kinematic_pose = *rt_reference_ptr_.readFromNonRT()->get();
    const auto desired_pose = desired_kinematic_pose.pose;

    Eigen::Vector3d desired_position(
      desired_pose.position.x, desired_pose.position.y, desired_pose.position.z);

    Eigen::Quaterniond desired_quaternion(
      desired_pose.orientation.w, desired_pose.orientation.x, desired_pose.orientation.y,
      desired_pose.orientation.z);
    desired_quaternion.normalize();

    Eigen::Vector3d actual_position = robot_data_.get()->oMf[end_effector_frame_].translation();
    Eigen::Matrix3d actual_orientation = robot_data_.get()->oMf[end_effector_frame_].rotation();

    pose_deviation_.head<3>() = desired_position - actual_position;
    pose_deviation_.tail<3>() =
      pinocchio::log3(desired_quaternion.toRotationMatrix() * actual_orientation.transpose());

    Vector6d desired_twist;
    desired_twist.head<3>() = Eigen::Vector3d(
      desired_kinematic_pose.pose_twist.linear.x, desired_kinematic_pose.pose_twist.linear.y,
      desired_kinematic_pose.pose_twist.linear.z);
    desired_twist.tail<3>() = Eigen::Vector3d(
      desired_kinematic_pose.pose_twist.angular.x, desired_kinematic_pose.pose_twist.angular.y,
      desired_kinematic_pose.pose_twist.angular.z);

    twist_deviation_ = desired_twist - actual_twist;

    desired_pose_accel_.head<3>() = Eigen::Vector3d(
      desired_kinematic_pose.pose_accel.linear.x, desired_kinematic_pose.pose_accel.linear.y,
      desired_kinematic_pose.pose_accel.linear.z);
    desired_pose_accel_.tail<3>() = Eigen::Vector3d(
      desired_kinematic_pose.pose_accel.angular.x, desired_kinematic_pose.pose_accel.angular.y,
      desired_kinematic_pose.pose_accel.angular.z);
  }
  else
  {
    pose_deviation_ = Vector6d::Zero();
    twist_deviation_ = Vector6d::Zero() - actual_twist;
    desired_pose_accel_ = Vector6d::Zero();
  }
}

void ImpedanceController::robot_description_param_cb(
  std::shared_future<std::vector<rclcpp::Parameter>> future)
{
  robot_urdf_ = future.get().at(0).as_string();
}

void ImpedanceController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn ImpedanceController::read_parameters()
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  degrees_of_freedom_ = params_.degrees_of_freedom;

  if (params_.stiffness.empty() || params_.damping.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Stiffness or damping array parameters were empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  taskspace_stiffness_.diagonal() = Vector6d(params_.stiffness.data());
  taskspace_damping_.diagonal() = Vector6d(params_.damping.data());

  if (std::fpclassify(params_.inertia[0]) == FP_ZERO)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "First element of the inertia param is 0.0. Inertia shaping disabled.");
    inertia_shaping_ = false;
  }
  else
  {
    taskspace_inertia_inv_ = Vector6d(params_.inertia.data()).cwiseInverse().asDiagonal();
    inertia_shaping_ = true;
  }

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
