"""Launch the three GNC nodes as independent processes."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("gnc_bringup"))
    nodes_share = Path(get_package_share_directory("gnc_ros"))
    parameters = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(share / "config" / "scenarios" / "figure_eight.yaml"),
            description="Scenario YAML overrides, applied after each node's defaults.",
        ),
        DeclareLaunchArgument("rviz", default_value="false", description="Open RViz."),
        Node(
            package="gnc_ros",
            executable="simulation_node",
            name="simulation_node",
            parameters=[str(nodes_share / "simulation" / "config" / "defaults.yaml"), parameters],
            output="screen",
        ),
        Node(
            package="gnc_ros",
            executable="estimation_node",
            name="estimation_node",
            parameters=[str(nodes_share / "estimation" / "config" / "defaults.yaml"), parameters],
            output="screen",
        ),
        Node(
            package="gnc_ros",
            executable="guidance_node",
            name="guidance_node",
            parameters=[str(nodes_share / "guidance" / "config" / "defaults.yaml"), parameters],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "simulation.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
