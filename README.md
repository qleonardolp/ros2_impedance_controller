# ros2_impedance_controller
[![IEEE Xplore](https://img.shields.io/badge/ICAR%202025-11338640-blue?logo=IEEE)](https://ieeexplore.ieee.org/document/11338640)

Robot impedance controller designed with the `ros2_control` framework and Pinocchio. Default branch ROS2 distro: Jazzy

- Custom build dependency: [kinematic_pose_msgs](https://github.com/qleonardolp/kinematic_pose_msgs)
- Simulation dependency: [ros2_descriptions](https://github.com/qleonardolp/ros2_descriptions)
- Handy reference generator: [impedance_reference_generator](https://github.com/qleonardolp/impedance_reference_generator)

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
