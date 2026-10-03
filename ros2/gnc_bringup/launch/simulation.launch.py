"""Launch the three GNC components with shared simulation-time settings."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = Path(get_package_share_directory("gnc_bringup"))
    parameters = str(share / "config" / "simulation.yaml")

    container = ComposableNodeContainer(
        name="gnc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="gnc_ros",
                plugin="gnc_ros::SimulationNode",
                name="simulation_node",
                parameters=[parameters],
            ),
            ComposableNode(
                package="gnc_ros",
                plugin="gnc_ros::EstimationNode",
                name="estimation_node",
                parameters=[parameters],
            ),
            ComposableNode(
                package="gnc_ros",
                plugin="gnc_ros::GuidanceNode",
                name="guidance_node",
                parameters=[parameters],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false", description="Open RViz."),
        container,
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "simulation.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
