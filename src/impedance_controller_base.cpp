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

#include "ros2_impedance_controller/impedance_controller_base.hpp"

namespace ros2_impedance_controller
{
ImpedanceControllerBase::ImpedanceControllerBase()
: controller_interface::ControllerInterface(),
  reference_subscriber_(nullptr),
  status_publisher_(nullptr),
  marker_publisher_(nullptr)
{
}

controller_interface::CallbackReturn ImpedanceControllerBase::on_init()
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

controller_interface::CallbackReturn ImpedanceControllerBase::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joint names is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }
  degrees_of_freedom_ = joint_names_.size();

  if (!configure_robot_model())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  auto qos_lowlatency = rclcpp::QoS(1);
  qos_lowlatency.best_effort().durability_volatile();
  qos_lowlatency.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

  status_publisher_ =
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/status", qos_lowlatency);

  status_rt_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      status_publisher_);

  if (visualize_reference_)
  {
    configure_visualization_marker();
    marker_publisher_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
      "~/reference_marker", rclcpp::SystemDefaultsQoS());
  }

  reference_subscriber_ = get_node()->create_subscription<ReferenceType>(
    "~/reference", qos_lowlatency,
    [this](const ReferenceType::SharedPtr msg)
    {
      rt_reference_.set(*msg);

      static uint16_t downsample = 0;
      if (visualize_reference_)
      {
        ++downsample;
        if (0 == downsample % 5)
        {
          marker_.pose = msg.get()->pose;
          marker_publisher_->publish(marker_);
          downsample = 0;
        }
      }
    });

  // Initialize dynamic Eigen members
  robot_q_.resize(degrees_of_freedom_);
  robot_dq_.resize(degrees_of_freedom_);
  robot_dq_last_.resize(degrees_of_freedom_);
  robot_ddq_.resize(degrees_of_freedom_);
  robot_ddq_filt_.resize(degrees_of_freedom_);
  robot_efforts_.resize(degrees_of_freedom_);
  effort_commands_.resize(degrees_of_freedom_);
  jacobian_ = Matrix6Xd::Zero(kCartesianDim, degrees_of_freedom_);

  custom_configuration();  // Derived class-specific configuration

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ImpedanceControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_config;
  command_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_)
  {
    command_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return command_config;
}

controller_interface::InterfaceConfiguration
ImpedanceControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_config;
  state_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_)
  {
    state_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    state_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    if (has_effort_states_)
    {
      state_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
  }
  return state_config;
}

controller_interface::CallbackReturn ImpedanceControllerBase::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  // Dynamic size members (joint space dim)
  jacobian_.setZero();
  effort_commands_.setZero();
  robot_ddq_filt_.setZero();
  robot_dq_last_.setZero();

  // ros2_control interfaces
  effort_command_interfaces_.resize(degrees_of_freedom_, nullptr);
  position_interfaces_.resize(degrees_of_freedom_, nullptr);
  velocity_interfaces_.resize(degrees_of_freedom_, nullptr);
  effort_interfaces_.resize(degrees_of_freedom_, nullptr);

  // Constant size members (task space dim)
  pose_deviation_.setZero();
  twist_deviation_.setZero();
  desired_pose_accel_.setZero();

  desired_quaternion_.setIdentity();
  desired_position_.setZero();
  desired_twist_.setZero();
  actual_pose_.setZero();
  actual_twist_.setZero();
  actual_accel_.setZero();

  for (const auto & interface : state_interfaces_)  // LoanedStateInterface from the base class
  {
    const std::string & joint_name = interface.get_prefix_name();
    const std::string & interface_type = interface.get_interface_name();

    auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
    if (it == joint_names_.end()) continue;
    const size_t index = std::distance(joint_names_.begin(), it);

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
    auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
    if (it == joint_names_.end()) continue;
    const size_t index = std::distance(joint_names_.begin(), it);
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

  custom_activation();  // Derived class-specific activation

  RCLCPP_WARN(logger, "Activated successfully!");

  clock_time_last_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceControllerBase::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  effort_command_interfaces_.clear();
  position_interfaces_.clear();
  velocity_interfaces_.clear();
  effort_interfaces_.clear();

  status_rt_publisher_->stop();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ImpedanceControllerBase::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  delta_t_ = static_cast<double>((time - clock_time_last_).nanoseconds()) * 1E-9;
  // Read state interfaces and update robot
  if (!update_robot())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to update robot model with state interfaces data");
    return controller_interface::return_type::ERROR;
  }

  update_deviation_and_reference();

  update_effort_commands();  // Derived class-specific control law

  for (uint8_t k = 0; k < degrees_of_freedom_; ++k)
  {
    if (!effort_command_interfaces_[k]->set_value(effort_commands_(k)))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }
  }

  publish_status();  // Derived class-specific diagnostic publisher
  clock_time_last_ = time;
  return controller_interface::return_type::OK;
}

bool ImpedanceControllerBase::configure_robot_model()
{
  std::string urdf_file =
    ament_index_cpp::get_package_share_directory(urdf_package_) + urdf_relative_path_;

  pinocchio::urdf::buildModel(urdf_file, robot_model_);
  robot_data_ = std::make_shared<pinocchio::Data>(robot_model_);

  if (!robot_model_.existFrame(end_effector_link_name_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Frame '%s' not found!", end_effector_link_name_.c_str());
    return false;
  }
  end_effector_frame_ = robot_model_.getFrameId(end_effector_link_name_);

  RCLCPP_INFO(
    get_node()->get_logger(), "Robot model %s loaded with %d DOFs", robot_model_.name.c_str(),
    robot_model_.nq);
  return true;
}

bool ImpedanceControllerBase::update_robot()
{
  for (uint8_t k = 0; k < degrees_of_freedom_; k++)
  {
    std::optional position = position_interfaces_[k]->get_optional();
    std::optional velocity = velocity_interfaces_[k]->get_optional();
    std::optional effort = effort_interfaces_[k]->get_optional();

    if (!position.has_value() || !velocity.has_value()) return false;

    if (has_effort_states_ && !effort.has_value()) return false;

    robot_q_(k) = position.value();
    robot_dq_(k) = velocity.value();
    if (has_effort_states_)
    {
      robot_efforts_(k) = effort.value();
    }

    // Finite difference
    robot_ddq_(k) = (robot_dq_(k) - robot_dq_last_(k)) / delta_t_;
    robot_dq_last_(k) = robot_dq_(k);
  }

  robot_ddq_filt_ = acc_lpf_alpha_ * robot_ddq_ + (1.0 - acc_lpf_alpha_) * robot_ddq_filt_;

  pinocchio::computeAllTerms(robot_model_, *robot_data_.get(), robot_q_, robot_dq_);
  // fill M(q)
  robot_data_->M.triangularView<Eigen::StrictlyLower>() =
    robot_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  return true;
}

void ImpedanceControllerBase::update_deviation_and_reference()
{
  pinocchio::computeFrameJacobian(
    robot_model_, *robot_data_.get(), robot_q_, end_effector_frame_, pinocchio::LOCAL_WORLD_ALIGNED,
    jacobian_);

  actual_twist_.noalias() = jacobian_ * robot_dq_;

  double position_norm1_ = 0;
  auto ref_optional = rt_reference_.try_get();

  if (ref_optional.has_value())
  {
    reference_ = ref_optional.value();
    position_norm1_ = abs(reference_.pose.position.x) + abs(reference_.pose.position.y) +
                      abs(reference_.pose.position.z);
  }

  if (ref_optional.has_value() && position_norm1_ > std::numeric_limits<double>::epsilon())
  {
    reference_ = ref_optional.value();

    desired_position_.x() = reference_.pose.position.x;
    desired_position_.y() = reference_.pose.position.y;
    desired_position_.z() = reference_.pose.position.z;

    desired_quaternion_.w() = reference_.pose.orientation.w;
    desired_quaternion_.x() = reference_.pose.orientation.x;
    desired_quaternion_.y() = reference_.pose.orientation.y;
    desired_quaternion_.z() = reference_.pose.orientation.z;
    desired_quaternion_.normalize();

    pose_deviation_.head<3>().noalias() =
      robot_data_.get()->oMf[end_effector_frame_].translation() - desired_position_;
    pose_deviation_.tail<3>().noalias() = pinocchio::log3(
      robot_data_.get()->oMf[end_effector_frame_].rotation() *
      desired_quaternion_.toRotationMatrix().transpose());

    desired_twist_(0) = reference_.pose_twist.linear.x;
    desired_twist_(1) = reference_.pose_twist.linear.y;
    desired_twist_(2) = reference_.pose_twist.linear.z;
    desired_twist_(3) = reference_.pose_twist.angular.x;
    desired_twist_(4) = reference_.pose_twist.angular.y;
    desired_twist_(5) = reference_.pose_twist.angular.z;

    twist_deviation_.noalias() = actual_twist_ - desired_twist_;

    desired_pose_accel_(0) = reference_.pose_accel.linear.x;
    desired_pose_accel_(1) = reference_.pose_accel.linear.y;
    desired_pose_accel_(2) = reference_.pose_accel.linear.z;
    desired_pose_accel_(3) = reference_.pose_accel.angular.x;
    desired_pose_accel_(4) = reference_.pose_accel.angular.y;
    desired_pose_accel_(5) = reference_.pose_accel.angular.z;
  }
  else
  {
    pose_deviation_.setZero();
    twist_deviation_.noalias() = actual_twist_;
    desired_pose_accel_.setZero();
  }
}

void ImpedanceControllerBase::configure_visualization_marker()
{
  marker_ = visualization_msgs::msg::Marker();
  marker_.header.frame_id = base_link_name_.c_str();
  marker_.ns = get_node()->get_name();
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

size_t ImpedanceControllerBase::get_dof() { return degrees_of_freedom_; }

}  // namespace ros2_impedance_controller
