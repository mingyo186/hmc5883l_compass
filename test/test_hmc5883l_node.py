# Copyright 2025 The hmc5883l_compass Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch test for HMC5883L compass sensor node."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import MagneticField
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch HMC5883L node in fake_mode for testing."""
    node = launch_ros.actions.Node(
        package='hmc5883l_compass',
        executable='hmc5883l_node.py',
        name='hmc5883l_compass_node',
        parameters=[{
            'fake_mode': True,
            'publish_rate': 50.0,
        }],
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {'sensor_node': node}


class TestHMC5883LTopics(unittest.TestCase):
    """Verify magnetic field topic publishes valid data."""

    def test_mag_topic_published(self):
        """Mag data topic should receive messages."""
        topic_list = [('mag/data', MagneticField)]
        with WaitForTopics(topic_list, timeout=10.0) as wait:
            self.assertEqual(wait.topics_received(), {'mag/data'})

    def test_mag_data_valid(self):
        """Verify MagneticField message has valid frame_id."""
        topic_list = [('mag/data', MagneticField)]
        with WaitForTopics(
            topic_list, timeout=10.0, messages_received_buffer_length=5
        ) as wait:
            msgs = wait.received_messages('mag/data')
            self.assertGreater(len(msgs), 0)
            for msg in msgs:
                self.assertEqual(msg.header.frame_id, 'mag_link')


class TestHMC5883LServices(unittest.TestCase):
    """Verify calibrate and reset services."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_hmc5883l_services')

    def tearDown(self):
        self.node.destroy_node()

    def test_calibrate_service(self):
        """Calibrate should return success in fake mode."""
        client = self.node.create_client(Trigger, 'mag/calibrate')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('fake', future.result().message.lower())
        self.node.destroy_client(client)

    def test_reset_service(self):
        """Reset should return success."""
        client = self.node.create_client(Trigger, 'mag/reset')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('reset complete', future.result().message.lower())
        self.node.destroy_client(client)


class TestHMC5883LParameters(unittest.TestCase):
    """Verify runtime parameter changes."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_hmc5883l_params')

    def tearDown(self):
        self.node.destroy_node()

    def test_change_publish_rate(self):
        """Publish_rate should be changeable at runtime."""
        client = self.node.create_client(
            SetParameters, 'hmc5883l_compass_node/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = SetParameters.Request()
        request.parameters = [
            Parameter('publish_rate', value=20.0).to_parameter_msg(),
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().results[0].successful)
        self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        """Node should exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
