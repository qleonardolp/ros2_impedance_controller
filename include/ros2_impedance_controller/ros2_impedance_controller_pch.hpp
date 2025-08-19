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

#ifndef ROS2_IMPEDANCE_CONTROLLER__ROS2_IMPEDANCE_CONTROLLER_PCH_HPP_
#define ROS2_IMPEDANCE_CONTROLLER__ROS2_IMPEDANCE_CONTROLLER_PCH_HPP_

// Precompile headers

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/frames.hpp"  // computeFrameJacobian
#include "pinocchio/parsers/urdf.hpp"      // ::urdf::buildModelFromXML

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
#include "realtime_tools/realtime_buffer.hpp"
#include "visualization_msgs/msg/marker.hpp"

#endif  // ROS2_IMPEDANCE_CONTROLLER__ROS2_IMPEDANCE_CONTROLLER_PCH_HPP_
