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

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"  // computeFrameJacobian
#include "pinocchio/parsers/urdf.hpp"      // ::urdf::buildModelFromXML

#include "ros2_impedance_controller/common_definitions.hpp"

namespace ros2_impedance_controller
{
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
   * @brief Prediction method. Run forward dynamics (ABA)
   * and integrate velocities up to the horizon.
   *
   * @param q    joint configuration vector
   * @param v    joint velocity vector
   * @param tau  joint torque vector
   */
  void predict(Eigen::VectorXd q, Eigen::VectorXd v, Eigen::VectorXd tau);

  /**
   * @brief Class destructor
   */
  ~TaskspacePredictor();

private:
  double timestep_;
  uint horizon_;
  pinocchio::Model robot_;
  pinocchio::FrameIndex ee_frame_;  // end effector frame
  std::shared_ptr<pinocchio::Data> data_;

  // Operational space inertia matrix (osim)
  Matrix6d actual_inertia_;

  Eigen::MatrixXd jacobian_;

  Eigen::VectorXd robot_q_;    // joint configuration
  Eigen::VectorXd robot_dq_;   // joint velocities
  Eigen::VectorXd robot_ddq_;  // joint acceleration
  Eigen::VectorXd zero_tau_;   // zero torque vector
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__TASKSPACE_PREDICTOR_HPP_
