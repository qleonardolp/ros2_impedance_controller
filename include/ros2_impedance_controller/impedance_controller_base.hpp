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

#ifndef ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROLLER_BASE_HPP_
#define ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROLLER_BASE_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"  // computeFrameJacobian
#include "pinocchio/parsers/urdf.hpp"      // ::urdf::buildModelFromXML

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kinematic_pose_msgs/msg/kinematic_pose.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "ros2_impedance_controller/common_definitions.hpp"

namespace ros2_impedance_controller
{
using ReferenceType = kinematic_pose_msgs::msg::KinematicPose;

const uint8_t kMaxJointSpaceDim = 48;  // avoid dynamic memory allocation

typedef Eigen::Matrix<double, kCartesianDim, Eigen::Dynamic, 0, kCartesianDim, kMaxJointSpaceDim>
  Matrix6Xd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxJointSpaceDim, 1> VectorXd;

// Default damping ratio
const double kDampingRatio = 1.00;

/**
 * \brief Cartesian impedance controllers base class for articulated robots.
 *
 * This class defines the minimal functionality for ros2_control
 * Cartesian impedance controllers. Claim only effort command interfaces.
 */
class ImpedanceControllerBase : public controller_interface::ControllerInterface
{
public:
  ImpedanceControllerBase();

  ~ImpedanceControllerBase() = default;

  // Indicating which command interfaces are to be claimed.
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // Indicating which state interfaces are to be claimed.
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * Derived controllers have to declare parameters in this method.
   */
  virtual void declare_parameters() = 0;

  /**
   * Derived controllers have to read parameters in this method and set `joint_names_`
   * variable and `has_effort_states_` variable. These variables are then used to propagate
   * the state and command interfaces configuration to controller manager.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and
   * their values are allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  virtual controller_interface::CallbackReturn read_parameters() = 0;

  /**
   * @brief Compute and update effort commands (control action).
   *
   * Before call this method, base class update robot states: `robot_positions_`,
   * `robot_velocities_`, `robot_accelerations_`, and `robot_efforts_` (if available).
   * Robot geometric Jacobian, kinematic deviations, and `reference_` are also updated.
   *
   * In the end, this method must write on `effort_commands_`.
   */
  virtual controller_interface::CallbackReturn update_effort_commands() = 0;

  /**
   * @brief Derived controllers can implement a status publication with this method.
   * Publisher `status_publisher_` message type is std_msgs::msg::Float64MultiArray.
   */
  virtual void publish_status() = 0;

  /**
   * @brief Derived controller custom configuration, called in on_configure().
   */
  virtual void custom_configuration() = 0;

  /**
   * @brief Derived controller custom activation, called in on_activate().
   */
  virtual void custom_activation() = 0;

  /**
   * @brief Update the robot data while running the controller.
   * State interfaces are fetch and the Forward Kinematics is computed.
   */
  bool update_robot();

  /**
   * @brief Update the end-effector pose and twist deviations (errors), and Jacobian.
   *
   * The position part is simply the reference minus the state.
   * The orientation part is computed using Lie algebra (pinocchio::log3),
   * free of gimbal lock errors. Twist deviation is =
   * desired_twist - \f$ J(q)*\dot{q} \f$. This method also updates
   * desired_pose_accel_.
   */
  void update_deviation_and_reference();

  /**
   * @brief Configure the internal robot model from the URDF available in the
   * parameter server ('robot_description'). In this way the URDF parsed by the
   * rigid-body dynamics library has been already parsed by ROS, resolving
   * launch-related parameter setting.
   *
   * @return true, in case of successful parsing;
   * @return false, in case of failed parsing or missing frame.
   */
  bool configure_robot_model();

  /**
   * @brief Configure reference marker (visualization_msgs::msg::Marker)
   * to be visualized on rviz.
   */
  void configure_visualization_marker();

  size_t get_dof();

  // Parameter defined members
  // These class members must be set through parameters.
  std::vector<std::string> joint_names_;
  std::string end_effector_link_name_;
  std::string base_link_name_;
  std::string urdf_package_;
  std::string urdf_relative_path_;
  bool has_effort_states_{false};
  bool visualize_reference_{false};
  // End of parameter defined members

  // Torque low-pass filter alpha for Nyquist frequency.
  double cmd_lpf_alpha_{0.7585469929947761};

  rclcpp::Time clock_time_last_;
  double delta_t_{0};

  // Impedance reference subscriber
  rclcpp::Subscription<ReferenceType>::SharedPtr reference_subscriber_;

  // TODO(@qleonardolp): update realtime containers
  // https://github.com/ros-controls/ros2_controllers/pull/1935
  realtime_tools::RealtimeThreadSafeBox<ReferenceType> rt_reference_;
  // Impedance reference
  ReferenceType reference_;

  // Controller status publisher. Useful for control analysis, logging or debug.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
    status_rt_publisher_;
  std_msgs::msg::Float64MultiArray status_msg_;

  // Publisher for reference visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  visualization_msgs::msg::Marker marker_;

  pinocchio::Model robot_model_;
  std::shared_ptr<pinocchio::Data> robot_data_;
  pinocchio::FrameIndex end_effector_frame_;

  // Desired stiffness
  DiagonalMatrix6d desired_stiffness_;
  // Desired damping
  DiagonalMatrix6d desired_damping_;
  // Inverse of the desired inertia
  DiagonalMatrix6d desired_inertia_inv_;
  // Desired inertia. Its the osim when not shaping inertia
  Matrix6d desired_inertia_;
  // Operational space inertia matrix (osim)
  Matrix6d actual_inertia_;

  // Joint to task space geometric Jacobian
  Matrix6Xd jacobian_;

  // Task space variables
  Vector6d actual_pose_;
  Vector6d actual_twist_;
  Vector6d actual_accel_;
  Vector6d desired_twist_;
  Vector6d desired_pose_accel_;  // Desired end effector twist derivative
  Eigen::Quaterniond desired_quaternion_;
  Eigen::Vector3d desired_position_;
  Vector6d pose_deviation_;   // End effector pose deviation
  Vector6d twist_deviation_;  // End effector twist deviation

  // Joint space state vectors
  VectorXd robot_positions_;
  VectorXd robot_velocities_;
  VectorXd robot_velocities_last_;
  VectorXd robot_accelerations_;
  VectorXd robot_efforts_;

  // Controller command vector (joint space)
  VectorXd effort_commands_;

private:
  // Position state interfaces
  std::vector<const hardware_interface::LoanedStateInterface *> position_interfaces_;
  // Velocity state interfaces
  std::vector<const hardware_interface::LoanedStateInterface *> velocity_interfaces_;
  // Effort state interfaces
  std::vector<const hardware_interface::LoanedStateInterface *> effort_interfaces_;
  // Effort command interfaces
  std::vector<hardware_interface::LoanedCommandInterface *> effort_command_interfaces_;

  size_t degrees_of_freedom_{1};
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROLLER_BASE_HPP_
