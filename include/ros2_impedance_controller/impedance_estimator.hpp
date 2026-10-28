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
#include <memory>
#include <qpOASES.hpp>

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

// Dynamic size row-major storage Eigen matrix,
// easing qpOASES C-style access to matrices. Use m.data()
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXr;

/**
 * \brief QP-based impedance params (M, D) estimator for adaptive control.
 *
 * This estimator compute the best inertia and damping for a rolling window
 * from `ddx`, `dx`, and `f_int - k*e`. It assumes that k is known.
 */
class ImpedanceEstimator
{
public:
  ImpedanceEstimator(uint window_size, uint axis, double delta_t);

  void init_sqproblem();

  /**
   * @brief Update FSM state according with thresholds.
   */
  int update_state(
    const Vector6d deviation, const Vector6d twist, const Vector6d accel, const Vector6d wrench);

  int update();

  /**
   * @brief Return solution
   */
  Eigen::Vector3d get_solution() { return u_qp_.head(kParamsDim); }

  uint_fast8_t get_fsm_state() { return static_cast<uint_fast8_t>(fsm_current_state_); }

  uint8_t get_counter() { return state_counter_; }

  bool get_record() { return record_updated_; }

private:
  /**
   * @brief Update QP Hessian matrix and gradient vector
   */
  void update_QP();

  void set_nullspace_bounds();

  void set_free_motion_bounds();

  void set_static_int_bounds();

  void set_equilibrium_bounds();

  // Class method pointer
  using Functor = void (ImpedanceEstimator::*)();

  // Dispatch Table
  static constexpr Functor bounds_table_[] = {
    &ImpedanceEstimator::set_nullspace_bounds,    // 0: NULLSPACE
    &ImpedanceEstimator::set_free_motion_bounds,  // 1: FREE_MOTION
    &ImpedanceEstimator::set_free_motion_bounds,  // 2: DYNAMIC_INT
    &ImpedanceEstimator::set_static_int_bounds,   // 3: STATIC_INT
    &ImpedanceEstimator::set_equilibrium_bounds   // 4: EQUILIBRIUM
  };

  bool debug_{true};

  uint window_;
  uint axis_;
  double delta_t_;

  State fsm_current_state_;  // Finite State Machine current state
  State fsm_last_state_;
  uint8_t state_counter_;

  // Online Active-Set strategy for QPs with varying matrices
  std::shared_ptr<qpOASES::SQProblem> sqproblem_;
  qpOASES::Options sqp_options_;  // QP Options
  qpOASES::returnValue sqp_ret_;  // QP return value
  int sqp_status_;                // QP simple status (see docs)
  uint constraints_dim_;          // constraints space dimension (nC): DoF
  uint action_dim_;               // action space dimension (nV): DoF*N

  MatrixXr H_qp_;           // QP Hessian matrix
  MatrixXr g_qp_;           // QP gradient vector
  MatrixXr A_qp_;           // QP constraint matrix
  Eigen::VectorXd lb_qp_;   // QP lower bound vector
  Eigen::VectorXd ub_qp_;   // QP upper bound vector
  Eigen::VectorXd lbA_qp_;  // QP lower constraints' bound vector
  Eigen::VectorXd ubA_qp_;  // QP upper constraints' bound vector
  Eigen::VectorXd du_max_;  // QP maximum action delta (u_{k+1} - u_{k})
  int nWSR_;                // QP max number of working set recalculations (nWSR)
  int nWSR_qp_;             // QP WSR for in place output in `hotstart` method
  double cputime_;          // QP maximum allowed CPU time in seconds
  double cputime_qp_;       // CPU time spent for QP solution (or to perform nWSR iterations)

  /**
   * @note Cost function is: min_{p} (V*u + b)^T*Q*(V*u + b),
   * where:
   * b := -q = k*e - f_int
   * V := [dde_k de_k 1 0 0 0 ...; 0 0 0 dde_{k-1} de_{k-1} 1 0 0 0 ...]
   */

  Eigen::MatrixXd V_qp_;  // `V` multiplies `u` in the cost function
  Eigen::VectorXd b_qp_;  // bias vector `b` in the cost function
  Eigen::MatrixXd Q_qp_;  // Norm weight
  Eigen::VectorXd u_qp_;  // QP solution

  /**
   * Concurrent Learning for Convergence in Adaptive Control without
   * Persistency of Excitation
   */

  std::shared_ptr<SlidingWindow> zspace_record_;  // Z-space recorded data [e de dde]
  Eigen::Matrix<double, 3, 3> zspace_rank_checking_;
  Eigen::FullPivLU<Eigen::Matrix3d> zspace_LU_;  // LU decomposition
  Eigen::Vector3d zspace_new_point_;
  bool record_updated_;
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_ESTIMATOR_HPP_
