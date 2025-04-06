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

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

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
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::INDIVIDUAL, command_interface_types_};
}

controller_interface::InterfaceConfiguration ImpedanceController::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
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

  if (!configure_robot_model())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  reference_subscriber_ = get_node()->create_subscription<ReferenceType>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const ReferenceType::SharedPtr msg) { rt_reference_ptr_.writeFromNonRT(msg); });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;

  bool interfaces_provided = controller_interface::get_ordered_interfaces(
    command_interfaces_,  // LoanedCommandInterface from the base class
    command_interface_types_, std::string(""), ordered_interfaces);

  if (!interfaces_provided || command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset reference buffer
  rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto new_end_effector_reference = rt_reference_ptr_.readFromRT();

  if (!new_end_effector_reference || !(*new_end_effector_reference))
  {
    return controller_interface::return_type::OK;
  }

  bool set_value_success = true;

  for (auto dof = 0ul; dof < command_interfaces_.size(); ++dof)
  {
    set_value_success = command_interfaces_[dof].set_value(0.0);
  }

  if (!set_value_success)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

bool ImpedanceController::configure_robot_model()
{
  dart::utils::DartLoader loader;
  std::string urdf_string;
  if (!get_node()->get_parameter("robot_description", urdf_string))  // FIX: is failing
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot description parameter");
    return false;
  }
  robot_skeleton_ = loader.parseSkeletonString(urdf_string, "");
  robot_base_ = robot_skeleton_->getBodyNode(params_.base_link);
  robot_end_effector_ = robot_skeleton_->getBodyNode(params_.interaction_link);

  if (!robot_base_ || !robot_end_effector_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find specified links in skeleton");
    return false;
  }
  degrees_of_freedom_ = static_cast<int>(robot_skeleton_->getNumDofs());
  RCLCPP_INFO(get_node()->get_logger(), "Robot model loaded with %d DOFs", degrees_of_freedom_);
  return true;
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

  command_interface_types_.clear();
  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/effort");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::ImpedanceController, controller_interface::ControllerInterface)
