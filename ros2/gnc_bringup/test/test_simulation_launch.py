"""Check the installed launch file, component loading, and interface wiring."""

from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rcl_interfaces.srv import GetParameters

from gnc_interfaces.srv import ResetSimulation


def generate_test_description():
    launch_file = (
        Path(get_package_share_directory("gnc_bringup")) / "launch" / "simulation.launch.py"
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSimulationLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("gnc_launch_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_components_and_topic_connections(self):
        expected_nodes = {"simulation_node", "estimation_node", "guidance_node"}
        expected_connections = {
            "/vehicle/state": (1, 2),
            "/target/ground_truth": (1, 0),
            "/radar/measurement": (1, 1),
            "/target/estimate": (1, 1),
            "/guidance/command": (1, 1),
        }
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            nodes_ready = expected_nodes.issubset(set(self.node.get_node_names()))
            topics_ready = all(
                self.node.count_publishers(topic) == publishers
                and self.node.count_subscribers(topic) == subscribers
                for topic, (publishers, subscribers) in expected_connections.items()
            )
            if nodes_ready and topics_ready:
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail("The launched components did not expose the expected topic connections.")

    def test_simulation_time_configuration(self):
        for name in ("simulation_node", "estimation_node", "guidance_node"):
            client = self.node.create_client(GetParameters, f"/{name}/get_parameters")
            try:
                self.assertTrue(client.wait_for_service(timeout_sec=10.0))
                request = GetParameters.Request(names=["use_sim_time"])
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
                self.assertTrue(future.done())
                self.assertTrue(future.result().values[0].bool_value)
            finally:
                self.node.destroy_client(client)

    def test_reset_reports_unimplemented(self):
        client = self.node.create_client(ResetSimulation, "/simulation/reset")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))
            future = client.call_async(ResetSimulation.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            self.assertTrue(future.done())
            self.assertFalse(future.result().success)
            self.assertIn("not implemented", future.result().message)
        finally:
            self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
