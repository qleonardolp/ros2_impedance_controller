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

#ifndef ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_ESTIMATOR_HPP_
#define ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_ESTIMATOR_HPP_

#include <sys/types.h>
#include <cstdint>

#include "ros2_impedance_controller/common_definitions.hpp"

namespace ros2_impedance_controller
{
const double kDeviationThreshold = 0.002;
const double kTwistDevThreshold = 0.010;
const double kAccelDevThreshold = 0.080;
const double kIntEffortThreshold = 1.00;

const uint8_t kParamsDim = 3;  // [k d m]

/**
 * @brief Finite State Machine enumeration.
 *
 * Classify the system dynamics between five (5) options:
 *
 * NULLSPACE: `e`, `de`, and `dde` (or `f_int`) = 0
 *
 * FREE_MOTION: `f_int` = 0
 *
 * DYNAMIC_INT: none = 0 (fallback for most of combinations)
 *
 * STATIC_INT: `de` and `dde` = 0
 *
 * EQUILIBRIUM: `e` and `de` = 0
 */
enum class State : uint_fast8_t
{
  NULLSPACE = 0,
  FREE_MOTION,
  DYNAMIC_INT,
  STATIC_INT,
  EQUILIBRIUM
};

/**
 * FSM Look Up Table (LUT)
 */
constexpr State StateLUT[16] = {
  State::DYNAMIC_INT,  // 0000
  State::FREE_MOTION,  // 0001
  State::DYNAMIC_INT,  // 0010
  State::FREE_MOTION,  // 0011
  State::DYNAMIC_INT,  // 0100
  State::FREE_MOTION,  // 0101
  State::STATIC_INT,   // 0110
  State::FREE_MOTION,  // 0111
  State::DYNAMIC_INT,  // 1000
  State::FREE_MOTION,  // 1001
  State::DYNAMIC_INT,  // 1010
  State::FREE_MOTION,  // 1011
  State::EQUILIBRIUM,  // 1100
  State::EQUILIBRIUM,  // 1101
  State::NULLSPACE,    // 1110: NULLSPACE
  State::NULLSPACE     // 1111: NULLSPACE (highest priority)
};

/**
 * \brief Concurrent Learning-based Z Estimator for Adaptive Control.
 */
class ImpedanceEstimator
{
public:
  ImpedanceEstimator(uint window_size, uint axis, double delta_t);

  /**
   * @brief Update FSM state according with thresholds.
   */
  int update_state(
    const Vector6d deviation, const Vector6d twist, const Vector6d accel, const Vector6d wrench);

  int update();

  /**
   * @brief Return solution
   */
  Eigen::Vector3d get_solution() { return zspace_new_point_; }

  uint_fast8_t get_fsm_state() { return static_cast<uint_fast8_t>(fsm_current_state_); }

  int get_index() { return subst_idx_; }

  double get_min_lambda() { return lowest_lambda_; }

private:
  bool debug_{true};

  uint window_;
  uint axis_;
  double delta_t_;

  State fsm_current_state_;  // Finite State Machine current state

  // State Variable Filter (2nd order low-pass)
  Eigen::Matrix2d A_svf_;  // SVF state space A
  Eigen::Vector2d B_svf_;  // SVF state space B

  Eigen::Vector2d e_state_;  // Using 'ddx' as the input
  Eigen::Vector2d fint_state_;

  Eigen::Vector2d de_state_;     // State derivative
  Eigen::Vector2d dfint_state_;  // State derivative

  /**
   * Concurrent Learning for Convergence in Adaptive Control
   * without Persistency of Excitation
   */
  uint_fast8_t num_stored_;
  int subst_idx_;  // Substitution index

  double fint_new_;       // f_{int} new sample (filtered)
  double lowest_lambda_;  // `H` lowest eigen value
  double min_lambda_;     // temporary min lambda

  Eigen::Vector3d zspace_new_point_;                         // zspace (e, de, dde) new point
  Eigen::Matrix3d zTz_new_;                                  // z.T * z (new)
  Eigen::Matrix3d H_;                                        // information matrix
  Eigen::Matrix3d H_temp_;                                   // Temporary information matrix
  Eigen::MatrixXd zspace_stack_;                             // zspace point time stack
  Eigen::VectorXd fint_stack_;                               // f_{int} time stack
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver_H_;  // H EigenSolver
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_ESTIMATOR_HPP_
