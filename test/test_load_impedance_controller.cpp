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

#include <gmock/gmock.h>

#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "test_asset.hpp"

TEST(TestLoadImpedanceController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_impedance_controller::valid_6dof_urdf, true, "test_controller_manager");
  const std::string test_file_path = std::string(TEST_FILES_DIRECTORY) + "/test_params.yaml";

  cm.set_parameter({"test_cartesian_controller.params_file", test_file_path});
  cm.set_parameter({"test_basic_controller.params_file", test_file_path});
  cm.set_parameter({"test_mpc_controller.params_file", test_file_path});

  cm.set_parameter(
    {"test_cartesian_controller.type", "ros2_impedance_controller/CartesianController"});
  cm.set_parameter(
    {"test_basic_controller.type", "ros2_impedance_controller/BasicCartesianController"});
  cm.set_parameter({"test_mpc_controller.type", "ros2_impedance_controller/MPCIController"});

  ASSERT_NE(cm.load_controller("test_cartesian_controller"), nullptr);
  ASSERT_NE(cm.load_controller("test_basic_controller"), nullptr);
  ASSERT_NE(cm.load_controller("test_mpc_controller"), nullptr);

  ASSERT_EQ(
    cm.configure_controller("test_cartesian_controller"), controller_interface::return_type::OK);
  ASSERT_EQ(
    cm.configure_controller("test_basic_controller"), controller_interface::return_type::OK);
  ASSERT_EQ(cm.configure_controller("test_mpc_controller"), controller_interface::return_type::OK);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
