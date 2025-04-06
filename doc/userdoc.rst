:github_url: https://github.com/ros-controls/ros2_controllers/blob/master/ros2_impedance_controller/doc/userdoc.rst

.. _ros2_impedance_controller_userdoc:

ros2_impedance_controller
==========================

This is a cartesian impedance controller, using the *ros2_controllers* and DART library.

Hardware interface type
-----------------------

This controller is intended to use *effort* hardware interfaces.


ROS 2 interface of the controller
---------------------------------

Topics
^^^^^^^

~/commands (input topic) [std_msgs::msg::Float64MultiArray]
  Target task space equilibrium point for the end effector.

Parameters
^^^^^^^^^^^^^^

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.

   .. tabs::

      .. group-tab:: ros2_impedance_controller

        .. generate_parameter_library_details:: ../src/ros2_impedance_controller_parameters.yaml
