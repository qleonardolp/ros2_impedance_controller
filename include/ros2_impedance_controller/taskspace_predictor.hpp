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

#ifndef ROS2_IMPEDANCE_CONTROLLER__TASKSPACE_PREDICTOR_HPP_
#define ROS2_IMPEDANCE_CONTROLLER__TASKSPACE_PREDICTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pinocchio/algorithm/aba.hpp"     // compute joint accelerations
#include "pinocchio/algorithm/crba.hpp"    // compute joint space inertia
#include "pinocchio/algorithm/frames.hpp"  // computeFrameJacobian
#include "pinocchio/algorithm/rnea.hpp"    // compute non linear terms (C, g)

#include "ros2_impedance_controller/common_definitions.hpp"

namespace ros2_impedance_controller
{
const uint8_t kStateSpaceDim = kCartesianDim * 2;

/**
 * \brief Task space dynamics predictor to support Model Predictive Cartesian Impedance Control.
 *
 * This class creates a copy of the robot model (pinocchio::Model) and integrate
 * its states along the desired horizon, `n` steps ahead. It applies `pinocchio::integrate`
 * method to perform sympletic integration in joint space, them compute the required
 * dynamics in task space.
 */
class TaskspacePredictor
{
public:
  /**
   * @brief Class constructor. The prediction total time is timestep * horizon.
   *
   * @param timestep  time step.
   * @param ee_frame  prediction horizon.
   * @param ee_frame  end effector frame name.
   * @param robot     valid pinocchio robot model.
   */
  TaskspacePredictor(double timestep, uint horizon, std::string ee_frame, pinocchio::Model robot);

  /**
   * @brief Class destructor
   */
  ~TaskspacePredictor();

  /**
   * @brief Prediction method. Run forward dynamics (ABA)
   * and integrate velocities up to the horizon.
   *
   * @param q    joint configuration vector
   * @param v    joint velocity vector
   * @param tau  joint torque vector
   */
  void predict(Eigen::VectorXd q, Eigen::VectorXd v, Eigen::VectorXd tau);

  /**
   * @brief Get Jacobian `stack` over the horizon
   */
  Eigen::MatrixXd get_Jn();

  /**
   * @brief Get Coriolis `stack` over the horizon
   */
  Eigen::MatrixXd get_Cn();

  /**
   * @brief Get state transition matrix over the horizon.
   * F dimensions: 12*`N` x 12
   */
  Eigen::MatrixXd get_F_matrix();

  /**
   * @brief Get input matrix over the horizon.
   * G dimensions: 12*`N` x DoF*`N`
   */
  Eigen::MatrixXd get_G_matrix();

  Eigen::VectorXd get_positions();

private:
  /**
   * @brief update the robot geometric Jacobian: J(q)
   */
  void update_jacobian();

  /**
   * @brief update task space Coriolis matrix (\f$ \Omega(x, dx) \f$)
   */
  void update_coriolis();

  /**
   * @brief update joint space inertia matrix. Pinocchio
   * only fills the upper triangular half of this matrix
   */
  void update_inertia();

  /**
   * @brief assemble the Jacobian horizon `stack`
   *
   * @param n  horizon time step
   */
  void assemble_Jn(std::size_t n);

  /**
   * @brief assemble the task space Coriolis horizon `stack`
   *
   * @param n  horizon time step
   */
  void assemble_Cn(std::size_t n);

  /**
   * @brief update state space transition matrix
   *
   * @param n  horizon time step
   */
  void update_Ak(std::size_t n);

  /**
   * @brief update state space input matrix
   *
   * @param n  horizon time step
   */
  void update_Bk(std::size_t n);

  /**
   * @brief assemble G matrix. This matrix stack the
   * system input from x_0 to x_{N-1}. The matrix is
   * lower triangular, but is not Toeplitz-shaped
   * because the system is time-varying.
   */
  void assemble_G();

  /**
   * @brief assemble F matrix. This matrix stack the
   * state transition from a initial state x_0 up to
   * x_{N-1} in the state space representation.
   */
  void assemble_F();

  double timestep_;
  uint horizon_;
  pinocchio::Model robot_;
  pinocchio::FrameIndex ee_frame_;  // end effector frame
  std::shared_ptr<pinocchio::Data> data_;

  // Task space inertia matrix (osim)
  Matrix6d task_inertia_;

  // Task space Coriolis
  Matrix6d task_coriolis_;

  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd jacobian_inv_;
  Eigen::MatrixXd jacobianT_inv_;
  Eigen::MatrixXd jacobian_dt_;  // Jacobian derivative

  Eigen::MatrixXd jacobianN_;  // Jacobian `stack` over the horizon
  Eigen::MatrixXd coriolisN_;  // Task Coriolis `stack` over the horizon

  Eigen::MatrixXd Ak_;  // State space transition matrix
  Eigen::MatrixXd Bk_;  // State space input matrix

  std::vector<Eigen::MatrixXd> AN_;  // State space A_k array over the horizon
  std::vector<Eigen::MatrixXd> BN_;  // State space B_k array over the horizon

  Eigen::MatrixXd F_;  // State space transition stack
  Eigen::MatrixXd G_;  // State space input stack

  Eigen::VectorXd robot_q_;    // joint configuration
  Eigen::VectorXd robot_dq_;   // joint velocities
  Eigen::VectorXd robot_ddq_;  // joint acceleration
  Eigen::VectorXd zero_tau_;   // zero torque vector

  Eigen::VectorXd robot_Q_;  // joint configuration over the horizon

  size_t dof_;

  bool debug_{false};
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__TASKSPACE_PREDICTOR_HPP_
