# ros2_impedance_controller

Robot impedance controller designed with the `ros2_control` framework and Pinocchio.

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
ros2 control set_controller_state impedance_controller active
```

### Simulation with Spot leg

```bash
ros2 launch ros2_impedance_controller simulation.launch.py robot:=spot_leg controller:=leg_impedance_controller
```

```bash
ros2 control set_controller_state leg_impedance_controller active
```

## About

The controller implement the classical impedance control law:

$$
\color{Black} \mathbf{\tau_{act}} = \mathbf{g}(\mathbf{q}) +
    \mathbf{J}(\mathbf{q})^{T}\,[\mathbf{\Lambda}(\mathbf{x})\,\ddot{\mathbf{x}}_{d} + \mathbf{\Omega}(\mathbf{x}, \dot{\mathbf{x}})\,\dot{\mathbf{x}}\,
    -\mathbf{\Lambda}(\mathbf{x})\,\mathbf{\Lambda_d}^{-1}\,(\mathbf{D_d}\,\dot{\mathbf{e}} + \mathbf{K_d}\,\mathbf{e}) +
    (\mathbf{\Lambda}(\mathbf{x})\,\mathbf{\Lambda_d}^{-1} - \mathbf{I})\,\mathbf{f_{int}}]
$$

The last term is optional. Without inertia shaping, the force-torque sensor feedback is not required. Then, the law simplifies to:

$$
\color{Black} \mathbf{\tau_{act}} = \mathbf{g}(\mathbf{q}) +
    \mathbf{J}(\mathbf{q})^{T}\,[\mathbf{\Lambda}(\mathbf{x})\,\ddot{\mathbf{x}}_{d} + \mathbf{\Omega}(\mathbf{x}, \dot{\mathbf{x}})\,\dot{\mathbf{x}}\,
    -\mathbf{D_d}\,\dot{\mathbf{e}} - \mathbf{K_d}\,\mathbf{e}]
$$

Please be aware that, without inertia shaping, the desired inertia is the robot current inertia.
Refer to Ott, C., 2008, _Cartesian Impedance Control of Redundant and Flexible-Joint Robots_ for further details.
