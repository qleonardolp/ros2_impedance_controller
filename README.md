# ros2_impedance_controller
[![IEEE Xplore](https://img.shields.io/badge/ICAR%202025-11338640-blue?logo=IEEE)](https://ieeexplore.ieee.org/document/11338640) [![Static Badge](https://img.shields.io/badge/v1.2.0-%20?style=flat&logo=github&labelColor=gray&color=blue)](https://github.com/qleonardolp/ros2_impedance_controller/tree/v1.2.0)


Robot impedance controller designed with the `ros2_control` framework and Pinocchio. Default branch ROS2 distro: Jazzy

1. Clone this repo;

2. Install build dependencies with:

```console
rosdep install --from-paths src -y --ignore-src
```

3. For simulations with Gazebo Harmonic, consider use my robot descriptions in [ros2_descriptions](https://github.com/qleonardolp/ros2_descriptions);

4. For a handful control reference signal generator use my package [robot_impedance_analyzer](https://github.com/qleonardolp/robot_impedance_analyzer/tree/v1.2.0).

## Installing qpOASES

The _Model Predictive Impedance Control_ (MPIC) relies on the `qpOASES` library to solve the QP problem.

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

## Instructions

### Simulation with Robotic Arm

Use launcher default arguments:

```bash
ros2 launch ros2_impedance_controller simulation.launch.py
```

```bash
ros2 control set_controller_state ur5_controller active
```

### Simulation with Spot leg

```bash
ros2 launch ros2_impedance_controller simulation.launch.py robot:=spot_leg controller:=spot_leg_controller
```

```bash
ros2 control set_controller_state spot_leg_controller active
```

### Simulation with Hydraulic Leg (HyL)

```bash
ros2 launch ros2_impedance_controller simulation.launch.py robot:=hyl controller:=hyl_controller
```

## About

The controller implement the classical impedance control law, following the notation from the book _Cartesian Impedance Control of Redundant and Flexible-Joint Robots_, Ott, C., 2008. Please check the [documentation](doc/ros2_impedance_controller_documentation.pdf) for further details.
