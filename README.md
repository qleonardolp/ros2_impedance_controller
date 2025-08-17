# ros2_impedance_controller

Robot impedance controller designed with the ros2_controllers framework and Pinocchio.

```bash
ros2 launch ros2_impedance_controller simulation.launch.py robot:=spot_leg controller:=leg_impedance_controller
```

```bash
ros2 control set_controller_state leg_impedance_controller active
```
