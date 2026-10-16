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
#include <memory>
#include <qpOASES.hpp>

#include "ros2_impedance_controller/common_definitions.hpp"

namespace ros2_impedance_controller
{
// Dynamic size row-major storage Eigen matrix,
// easing qpOASES C-style access to matrices. Use m.data()
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXr;

const uint8_t kParamsDim = 3;

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

  int update(const Vector6d accel, const Vector6d twist, const Vector6d wrench);

  /**
   * @brief Return solution
   */
  Eigen::Vector3d get_solution() { return u_qp_.head(kParamsDim); }

private:
  bool debug_{true};

  uint window_;
  uint axis_;
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
};

}  // namespace ros2_impedance_controller

#endif  // ROS2_IMPEDANCE_CONTROLLER__IMPEDANCE_ESTIMATOR_HPP_
