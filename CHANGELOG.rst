^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_impedance_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-08-18)
------------------
* Support N-DoF robots. Tested with 6-DoF arm and 3-DoF leg
* Load robot URDF from `urdf_package` and `urdf_relative_path` params
* Migrate robot description to `ros2_descriptions <https://github.com/qleonardolp/ros2_descriptions>`_
* Derive the damping gains from the desired stiffness and mass when using inertia shaping
* Add Port-Hamiltonian based diagnostics
* Add command torque low-pass filter
* Add control law documentation on README

0.0.2 (2025-04-06)
------------------
* Add UR5 manipulator description files and launchers for simulation and visualization

0.0.1 (2025-04-05)
------------------
* Add folders and first files, such as package.xml, CMakeLists, and .clang-format.
* Adapt userdoc.rst from `forward_command_controller`
* Contributors: Leonardo F. dos Santos
