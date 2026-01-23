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
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace ros2_impedance_controller
{
using ReferenceType = kinematic_pose_msgs::msg::KinematicPose;

const uint8_t kCartesianSpaceDim = 6;

using DiagonalMatrix6d = Eigen::DiagonalMatrix<double, kCartesianSpaceDim>;
using Vector6d = Eigen::Matrix<double, kCartesianSpaceDim, 1>;
using Matrix6d = Eigen::Matrix<double, kCartesianSpaceDim, kCartesianSpaceDim>;

const uint8_t kMaxJointSpaceDim = 12;  // avoiding dynamic memory allocation

typedef Eigen::Matrix<
  double, kCartesianSpaceDim, Eigen::Dynamic, 0, kCartesianSpaceDim, kMaxJointSpaceDim>
  Matrix6Xd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxJointSpaceDim, 1> VectorXd;

// Task space generalized inertia matrix eigenvalues
const Vector6d kDefaultInertia(0.000220625, 0.00256287, 0.00588485, 3.82715, 10.2802, 131.032);

// Default damping ratio
const double kDampingRatio = 1.00;

/**
 * \brief Cartesian impedance controller for articulated robots.
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

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
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
   * @brief Update the robot data while running the controller.
   * State interfaces are fetch and the Forward Kinematics is computed.
   */
  bool update_robot();

  /**
   * @brief Update the end-effector pose and twist deviations (errors).
   * The position part is simply the reference minus the state.
   * The orientation part is computed using Lie algebra (pinocchio::log3),
   * free of gimbal lock errors. Twist deviation is =
   * desired_twist - \f$ J(q)*\dot{q} \f$. This method also updates
   * desired_pose_accel_.
   */
  void update_deviations();

  /**
   * @brief Impedance space diagnostics with pose_deviation_, twist_deviation_,
   * and interaction wrench estimation. Publisher reuse the KinematicPose type.
   */
  virtual void diagnostics() = 0;

  /**
   * @brief Read simple parameters, as such link (frame) names and
   * number of joints.
   */
  virtual controller_interface::CallbackReturn read_parameters() = 0;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

private:
  void configure_visualization_marker();

  std::vector<const hardware_interface::LoanedStateInterface *> position_interfaces_;
  std::vector<const hardware_interface::LoanedStateInterface *> velocity_interfaces_;
  std::vector<const hardware_interface::LoanedStateInterface *> effort_interfaces_;
  std::vector<hardware_interface::LoanedCommandInterface *> effort_command_interfaces_;

  bool has_effort_states_{true};
  bool inertia_shaping_{false};
  bool debug_logger_{true};

  std::string base_link_;
  std::string interaction_link_;
  size_t degrees_of_freedom_{1};

  rclcpp::Time clock_time_last_;
  double ellapsed_time_{0};
  // Torque low-pass filter alpha for Nyquist frequency.
  double cmd_lpf_alpha_{0.7585469929947761};

  /* Port-Hamiltonian variables */
  // Impedance power
  double impedance_power_{0};
  // Impedance Hamiltonian function value
  double hamiltonian_{0};
  double hamiltonian_last_{0};
  double hamiltonian_filtered_{0};
  double hamiltonian_derivative_{0};

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
  // Desired impedance wrench [forces, torques].T
  Vector6d impedance_wrench_;
  // Interaction wrench [forces, torques].T
  Vector6d sensor_wrench_;
  // Interaction wrench estimation
  Vector6d estimated_wrench_;
  // End effector pose deviation
  Vector6d pose_deviation_;
  // End effector twist deviation
  Vector6d twist_deviation_;

  // update_deviations variables (task space)
  ReferenceType desired_kpose_;
  Vector6d actual_pose_;
  Vector6d actual_twist_;
  Vector6d actual_accel_;
  Vector6d desired_twist_;
  Eigen::Vector3d desired_position_;
  Eigen::Quaterniond desired_quaternion_;

  // Joint space state vectors
  VectorXd robot_positions_;
  VectorXd robot_velocities_;
  VectorXd robot_velocities_last_;
  VectorXd robot_accelerations_;
  VectorXd robot_efforts_;

  // Controller effort command vector (joint space)
  VectorXd effort_commands_;
  VectorXd commands_filtered_;

  Matrix6Xd jacobian_;

  // Controller Reference Subscriber
  realtime_tools::RealtimeThreadSafeBox<ReferenceType> rt_reference_;
  // TODO(@qleonardolp): update realtime containers
  // https://github.com/ros-controls/ros2_controllers/pull/1935
  rclcpp::Subscription<ReferenceType>::SharedPtr reference_subscriber_;

  // Interaction force subscriber
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr interaction_subscriber_;

  // Deviation, deviation derivative and interaction wrench publisher (for impedance space)
  // Interaction wrench goes on the accel field, replacing accelerations by forces and torques.
  rclcpp::Publisher<ReferenceType>::SharedPtr status_publisher_;

  // Publisher for reference visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  visualization_msgs::msg::Marker marker_;
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROLLER_BASE_HPP_
