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
            "gz_gui",
            default_value="false",
            description="Start Gazebo GUI. The default behavior"
            + " starts gazebo in server mode using Rviz2 as graphical interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="legged_benchmark",
            description="Gazebo world. See worlds directory.",
        )
    )

    # Arguments variables
    gz_gui = LaunchConfiguration("gz_gui")
    package_share = FindPackageShare("ros2_impedance_controller")
    gazebo_world = PathJoinSubstitution(
        [package_share, "worlds", [LaunchConfiguration("world"), ".sdf"]]
    )
    bridges = PathJoinSubstitution([package_share, "config", "ros_gz_bridge_clock_only.yaml"])
    controllers_config = PathJoinSubstitution([package_share, "config", "controllers.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "config", "spot_rviz.rviz"])

    spot_controllers = [
        "spot_fl_control",
        "spot_fr_control",
        "spot_hl_control",
        "spot_hr_control",
    ]

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": ["-r -v1 ", gazebo_world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(gz_gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": ["-s -r -v0 ", gazebo_world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(gz_gui),
    )

    # ROS-Gazebo bridges
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridges}],
        output="screen",
    )

    # Get URDF via xacro
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_descriptions"),
                    "description",
                    "spot.urdf.xacro",
                ]
            ),
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_urdf}],
    )

    # Robot spawner in Gazebo.
    # This node indirectly uses the robot_urdf parsed here,
    # through the topic /robot_description
    gz_entity_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-topic",
            "/robot_description",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            *spot_controllers,
            "--inactive",
            "--param-file",
            controllers_config,
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
        gz_entity_spawner,
        joint_state_broadcaster_spawner,
        controllers_spawner,
        rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)
