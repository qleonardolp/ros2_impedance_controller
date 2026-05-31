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

#ifndef TEST_ASSET_HPP_
#define TEST_ASSET_HPP_

namespace ros2_impedance_controller
{
const auto valid_6dof_urdf =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="b7">
  <link name="base_link">
    <inertial>
      <mass value="1.850"/>
      <origin rpy="0 0 0" xyz="0 0 0.015"/>
      <inertia
        ixx="0.0025280" ixy="0.0" ixz="0.0"
        iyy="0.0025930" iyz="0.0"
        izz="0.0033120"/>
    </inertial>
  </link>

  <joint name="g_joint" type="revolute">
    <parent link="base_link"/>
    <child link="1013_base_link"/>
    <origin xyz="0.0665 0 0.0858" rpy="0 0 3.14159265359"/>
    <axis xyz="0 0 -1"/>
    <limit effort="110.0" velocity="0.7854" lower="-6.28318530718" upper="6.28318530718"/>
    <dynamics damping="7.22" friction="0.079"/>
  </joint>

  <link name="1013_base_link">
    <inertial>
      <mass value="1.620"/>
      <origin rpy="0 0 0" xyz="0.031 -0.010 0.098"/>
      <inertia
        ixx="0.003180" ixy="0.0" ixz="0.0"
        iyy="0.004209" iyz="0.0"
        izz="0.002387"/>
    </inertial>
  </link>

  <joint name="f_joint" type="revolute">
    <parent link="1013_base_link"/>
    <child link="1013_joint_link"/>
    <origin xyz="0.046 0 0.0667" rpy="0 0 0"/>
    <axis xyz="0 -1 0"/>
    <limit effort="110.0" velocity="0.7854" lower="0.0" upper="3.2"/>
    <dynamics damping="13.90" friction="0.079"/>
  </joint>

  <link name="1013_joint_link">
    <inertial>
      <mass value="2.120"/>
      <origin rpy="0 0 0" xyz="0.022 0.004 -0.176"/>
      <inertia
        ixx="0.035166" ixy="0.0" ixz="0.0"
        iyy="0.035981" iyz="0.0"
        izz="0.003420"/>
    </inertial>
  </link>

  <joint name="e_joint" type="revolute">
    <parent link="1013_joint_link"/>
    <child link="1026_0_joint_link"/>
    <origin xyz="0.0052 0 -0.29434" rpy="3.14159265359 0 3.14159265359"/>
    <axis xyz="0 -1 0"/>
    <limit effort="110.0" velocity="0.7854" lower="0.0" upper="3.2"/>
    <dynamics damping="10.18" friction="0.079"/>
  </joint>

  <link name="1026_0_joint_link">
    <inertial>
      <mass value="1.330"/>
      <origin rpy="0 0 0" xyz="0.034 -0.003 -0.046"/>
      <inertia
        ixx="0.001809" ixy="0.0" ixz="0.0"
        iyy="0.002024" iyz="0.0"
        izz="0.001557"/>
    </inertial>
  </link>

  <joint name="d_joint" type="revolute">
    <parent link="1026_0_joint_link"/>
    <child link="1023_0_base_link"/>
    <origin xyz="0.0408 0 -0.0956" rpy="3.14159265359 0 0"/>
    <axis xyz="0 0 -1"/>
    <limit effort="110.0" velocity="0.7854" lower="-6.28318530718" upper="6.28318530718"/>
    <dynamics damping="7.13" friction="0.079"/>
  </joint>

  <link name="1023_0_base_link">
    <inertial>
      <mass value="1.330"/>
      <origin rpy="0 0 0" xyz="-0.028 0.012 0.050"/>
      <inertia
        ixx="0.001983" ixy="0.0" ixz="0.0"
        iyy="0.002676" iyz="0.0"
        izz="0.001526"/>
    </inertial>
  </link>

  <joint name="c_joint" type="revolute">
    <parent link="1023_0_base_link"/>
    <child link="1023_0_joint_link"/>
    <origin xyz="-0.0408 0 0.0644" rpy="0 0 3.14159265359"/>
    <axis xyz="0 -1 0"/>
    <limit effort="110.0" velocity="0.7854" lower="0.0" upper="3.2"/>
    <dynamics damping="9.59" friction="0.079"/>
  </joint>

  <link name="1023_0_joint_link">
    <inertial>
      <mass value="0.970"/>
      <origin rpy="0 0 0" xyz="0.031 0.006 -0.039"/>
      <inertia
        ixx="0.001272" ixy="0.0" ixz="0.0"
        iyy="0.001511" iyz="0.0"
        izz="0.001295"/>
    </inertial>
  </link>

  <joint name="b_joint" type="revolute">
    <parent link="1023_0_joint_link"/>
    <child link="1038_base_link"/>
    <origin xyz="0.0408 0 -0.0956" rpy="3.14159265359 0 3.14159265359"/>
    <axis xyz="0 0 -1"/>
    <limit effort="20.0" velocity="0.7854" lower="-6.28318530718" upper="6.28318530718"/>
    <dynamics damping="3.78" friction="0.079"/>
  </joint>

  <link name="1038_base_link">
    <inertial>
      <mass value="1.090"/>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.127"/>
      <inertia
        ixx="0.001977" ixy="0.0" ixz="0.0"
        iyy="0.001986" iyz="0.0"
        izz="0.001035"/>
    </inertial>
  </link>

  <joint name="1038_end_joint" type="fixed">
    <parent link="1038_base_link"/>
    <child link="jaws_base_link"/>
    <origin xyz="0 0 0.13075" rpy="0 -1.57079632679 0"/>
  </joint>

  <link name="jaws_base_link"/>

  <ros2_control name="B7System" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>

    <joint name="g_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="f_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="e_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="d_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="c_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <joint name="b_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)";

}  // namespace ros2_impedance_controller

#endif  // TEST_ASSET_HPP_
