# ros2_impedance_controller

[![IEEE Xplore](https://img.shields.io/badge/ICAR%202025-11338640-blue?logo=IEEE)](https://ieeexplore.ieee.org/document/11338640) [![Static Badge](https://img.shields.io/badge/v1.2.0-%20?style=flat&logo=github&labelColor=gray&color=blue)](https://github.com/qleonardolp/ros2_impedance_controller/tree/v1.2.0)

Robot Cartesian impedance controller based on the `ros2_control` framework and Pinocchio. Default branch ROS2 distro: `Jazzy`

The `ros2_impedance_controller` is meant to be a robot-agnostic, fully ROS2 ecosystem impedance controller, with diagnostics for researchers.
<!-- There are many Cartesian (task space) impedance controllers, mainly for ROS 1, but ... -->

## Installation

1. Clone this repo;

2. Install build dependencies with: `rosdep install --from-paths src -y --ignore-src`

## Features

By making a slight modification to your URDF, you can use the impedance controller with any rigid-body leg or manipulator. Check the URDF section in the [documentation](doc/ros2_impedance_controller_documentation.pdf) to understand _how_ and _why_ adequate your robot description to use with the available controllers. For a quick first try with Gazebo Harmonic, consider using my robot descriptions in [ros2_descriptions](https://github.com/qleonardolp/ros2_descriptions), and my simulation settings in [robot_impedance_lab](https://github.com/qleonardolp/robot_impedance_lab).

According to the classical impedance definitions, the controller input is the end-effector pose and its derivatives. For easy standardization, this input type is the [`kinematic_pose_msgs`](https://github.com/qleonardolp/kinematic_pose_msgs). The package [robot_impedance_analyzer](https://github.com/qleonardolp/robot_impedance_analyzer/) can be used for control analysis with single-axis parametric inputs such as step, sine and square waves, PRBS and others.

## Installing qpOASES (optional)

The _Model Predictive Cartesian Impedance Controller_ (MPCIC) uses `qpOASES` to solve the QP problem.

> [!WARNING]
> MPCIC is in development phase.

Clone and checkout the tag:

  ```bash
  git clone https://github.com/coin-or/qpOASES.git
  cd qpOASES && git checkout releases/3.2.2
  ```

Build and install:

  ```bash
  mkdir build && cd build
  cmake .. -D CMAKE_CXX_FLAGS="-fPIC"
  make
  sudo make install
  ```

## About

The controllers implement Hogan's classical impedance control law, following the notation from the book _Cartesian Impedance Control of Redundant and Flexible-Joint Robots_, Ott, C., 2008. Please check the [documentation](doc/ros2_impedance_controller_documentation.pdf) for further details.
