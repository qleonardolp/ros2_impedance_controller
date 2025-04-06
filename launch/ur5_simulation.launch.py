# Copyright 2025 qleonardolp
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Attach gdbserver to the controller manager node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_gui",
            default_value="false",
            description="Start Gazebo GUI. The default behavior \
            starts gazebo in server mode using Rviz2 as graphical interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            default_value="ur5.urdf.xacro",
            description="Robot model.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="true",
        )
    )

    # Initialize Arguments
    debug = LaunchConfiguration("debug")
    robot_model = LaunchConfiguration("robot")
    gz_gui = LaunchConfiguration("gz_gui")

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=IfCondition(gz_gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
        condition=UnlessCondition(gz_gui),
    )

    # Gazebo bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="log",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "ur5",
            "-allow_renaming",
            "true",
        ],
    )

    # Get URDF via xacro
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_impedance_controller"), "description", "urdf", robot_model]
            ),
            " ",
            "use_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_urdf}

    controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_impedance_controller"),
            "config",
            "controllers.yaml",
        ]
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ros2_impedance_controller"), "config", "rviz2.rviz"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        output="both",
        emulate_tty=True,
        remappings=[("~/robot_description", "/robot_description")],
        prefix=["gdbserver localhost:3000"],
        condition=IfCondition(debug),
    )

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "ros2_impedance_controller",
            "--param-file",
            controllers,
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=UnlessCondition(gz_gui),
    )

    nodes = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        robot_state_publisher,
        gz_spawn_entity,
        controller_manager,
        controllers_spawner,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)
