import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simulator = LaunchConfiguration("use_simulator")
    world = LaunchConfiguration("world")

    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")

    declare_simulator_cmd = DeclareLaunchArgument(
        name="headless",
        default_value="true",
        description="Whether to execute gzclient",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name="use_simulator",
        default_value="True",
        description="Whether to start the simulator",
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        condition=IfCondition(use_simulator),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)

    return ld
