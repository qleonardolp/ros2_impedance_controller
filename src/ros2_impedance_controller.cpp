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

#include <chrono>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
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
    controller_interface::interface_configuration_type::ALL};
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

  reference_subscriber_ = get_node()->create_subscription<ReferenceType>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const ReferenceType::SharedPtr msg) { rt_reference_ptr_.writeFromNonRT(msg); });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_cmd_interfaces;

  bool interfaces_provided = controller_interface::get_ordered_interfaces(
    command_interfaces_,  // LoanedCommandInterface from the base class
    command_interface_types_, std::string(""), ordered_cmd_interfaces);

  if (!interfaces_provided)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_cmd_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    ordered_state_interfaces;

  size_t minimal_states_size = params_.joints.size() * 2;  // positions and velocities

  interfaces_provided = controller_interface::get_ordered_interfaces(
    state_interfaces_,  // LoanedStateInterface from the base class
    state_interface_types_, std::string(""), ordered_state_interfaces);

  size_t ordered_states_size = ordered_state_interfaces.size();

  has_effort_states_ = !(ordered_states_size == minimal_states_size);

  if (
    ordered_states_size != state_interface_types_.size() &&
    ordered_states_size != minimal_states_size)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu state interfaces or %zu without effort, got %zu",
      state_interface_types_.size(), minimal_states_size, ordered_state_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset reference buffer
  rt_reference_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReferenceType>>(nullptr);

  RCLCPP_WARN(get_node()->get_logger(), "activate successful");
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
  // TODO(myself): Read state_interfaces_[0].get_optional() ...

  auto new_end_effector_reference = rt_reference_ptr_.readFromRT();

  if (!new_end_effector_reference || !(*new_end_effector_reference))
  {
    return controller_interface::return_type::OK;
  }

  bool set_value_success = true;

  // TODO(myself): check order to match the joints
  for (auto & command_interface : command_interfaces_)
  {
    set_value_success = command_interface.set_value(0.0);  // is working
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
  std::string share_dir = ament_index_cpp::get_package_share_directory("ros2_impedance_controller");
  loader.addPackageDirectory("ros2_impedance_controller", share_dir);

  parameters_client_->wait_for_service();
  auto parameters_future = parameters_client_->get_parameters(
    {"robot_description"},
    std::bind(&ImpedanceController::robot_description_param_cb, this, std::placeholders::_1));

  parameters_future.wait_for(std::chrono::seconds(5));  // TODO(myself): fix determinism

  robot_skeleton_ = loader.parseSkeletonString(robot_urdf_, "");
  robot_base_ = robot_skeleton_->getBodyNode(params_.base_link);
  robot_end_effector_ = robot_skeleton_->getBodyNode(params_.interaction_link);

  if (!robot_base_ || !robot_end_effector_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find specified links in skeleton");
    return false;
  }
  degrees_of_freedom_ = robot_skeleton_->getNumDofs();
  RCLCPP_INFO(get_node()->get_logger(), "Robot model loaded with %zu DOFs", degrees_of_freedom_);
  return true;
}

bool ImpedanceController::update_robot_model_states()
{
  bool success = true;
  for (auto & state_interface : state_interfaces_)
  {
    auto joint_name = state_interface.get_prefix_name();
    auto interface_type = state_interface.get_interface_name();

    std::optional state = state_interface.get_optional();

    if (state.has_value())
    {
      // use a map...
      if (interface_type.c_str() == hardware_interface::HW_IF_POSITION)
      {
        robot_skeleton_->getDof(joint_name)->setPosition(state.value());
      }
    }
    else
    {
      success = success && false;
    }
  }
  return success;
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

  state_interface_types_.clear();
  command_interface_types_.clear();
  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    state_interface_types_.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    state_interface_types_.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interface_types_.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace ros2_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_impedance_controller::ImpedanceController, controller_interface::ControllerInterface)
