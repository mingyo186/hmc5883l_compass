#!/usr/bin/env python3
"""ROS2 node that reads HMC5883L over I2C and publishes sensor_msgs/MagneticField."""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import MagneticField
from std_srvs.srv import Trigger

from hmc5883l_compass.hmc5883l_driver import HMC5883LDriver, FakeHMC5883LDriver


class HMC5883LCompassNode(Node):
    def __init__(self):
        super().__init__('hmc5883l_compass_node')

        # ── Declare parameters ────────────────────────────────────
        self.declare_parameter('fake_mode', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x1E)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('frame_id', 'mag_link')
        self.declare_parameter('gain', 1)
        self.declare_parameter('data_rate', 4)
        self.declare_parameter('samples_averaged', 3)
        self.declare_parameter('magnetic_field_covariance', 0.0)

        # ── Read parameters ───────────────────────────────────────
        self.fake_mode = self.get_parameter('fake_mode').value
        self.bus_num   = self.get_parameter('i2c_bus').value
        self.address   = self.get_parameter('device_address').value
        rate           = self.get_parameter('publish_rate').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.gain      = self.get_parameter('gain').value
        self.data_rate = self.get_parameter('data_rate').value
        self.samples_avg = self.get_parameter('samples_averaged').value
        self.mag_cov   = self.get_parameter('magnetic_field_covariance').value

        # ── Mag bias (set by calibration) ─────────────────────────
        self.mag_bias = [0.0, 0.0, 0.0]

        # ── Initialise driver ─────────────────────────────────────
        self._init_driver()

        # ── Publisher + timer ─────────────────────────────────────
        self.pub = self.create_publisher(MagneticField, 'mag/data', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'Publishing sensor_msgs/MagneticField on "mag/data" @ {rate} Hz')

        # ── Services ──────────────────────────────────────────────
        self.create_service(Trigger, 'mag/calibrate', self._calibrate_cb)
        self.create_service(Trigger, 'mag/reset', self._reset_cb)
        self.get_logger().info(
            'Services: "mag/calibrate", "mag/reset"')

        # ── Parameter change callback ─────────────────────────────
        self.add_on_set_parameters_callback(self._on_param_change)

    # ── Driver init helper ───────────────────────────────────────
    def _init_driver(self):
        if self.fake_mode:
            self.driver = FakeHMC5883LDriver()
            self.get_logger().info(
                'FAKE MODE enabled — generating random magnetic field data')
        else:
            try:
                self.driver = HMC5883LDriver(
                    self.bus_num, self.address,
                    self.gain, self.data_rate, self.samples_avg)
                id_str = self.driver.id_string()
                self.get_logger().info(
                    f'HMC5883L initialised  bus={self.bus_num}  '
                    f'addr=0x{self.address:02X}  ID={id_str}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open HMC5883L: {e}')
                raise

    # ── Timer callback ───────────────────────────────────────────
    def _timer_cb(self):
        try:
            mx, my, mz = self.driver.read_all()
        except OSError as e:
            self.get_logger().warn(
                f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

        # Apply hard-iron bias correction
        mx -= self.mag_bias[0]
        my -= self.mag_bias[1]
        mz -= self.mag_bias[2]

        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.magnetic_field.x = mx
        msg.magnetic_field.y = my
        msg.magnetic_field.z = mz
        mc = self.mag_cov
        msg.magnetic_field_covariance = [
            mc,  0.0, 0.0,
            0.0, mc,  0.0,
            0.0, 0.0, mc,
        ]
        self.pub.publish(msg)

    # ── Service: /mag/calibrate ──────────────────────────────────
    def _calibrate_cb(self, request, response):
        if self.fake_mode:
            response.success = True
            response.message = 'Calibration complete (fake)'
            self.get_logger().info('Calibration requested in fake mode — skipped')
            return response

        self.get_logger().info(
            'Calibrating mag — collecting data for 2 seconds...')
        samples = []
        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            try:
                mx, my, mz = self.driver.read_all()
                samples.append((mx, my, mz))
            except OSError:
                pass
            time.sleep(0.02)

        if not samples:
            response.success = False
            response.message = 'Calibration failed — no samples collected'
            return response

        n = len(samples)
        self.mag_bias[0] = sum(s[0] for s in samples) / n
        self.mag_bias[1] = sum(s[1] for s in samples) / n
        self.mag_bias[2] = sum(s[2] for s in samples) / n

        response.success = True
        response.message = (
            f'Calibration complete — {n} samples, '
            f'bias=({self.mag_bias[0]:.9f}, '
            f'{self.mag_bias[1]:.9f}, {self.mag_bias[2]:.9f}) T')
        self.get_logger().info(response.message)
        return response

    # ── Service: /mag/reset ──────────────────────────────────────
    def _reset_cb(self, request, response):
        self.mag_bias = [0.0, 0.0, 0.0]
        self.driver.close()
        self._init_driver()

        response.success = True
        response.message = 'Sensor reset complete'
        self.get_logger().info(response.message)
        return response

    # ── Runtime parameter change ─────────────────────────────────
    def _on_param_change(self, params):
        for param in params:
            if param.name == 'publish_rate':
                new_rate = param.value
                if new_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_rate must be > 0')
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / new_rate, self._timer_cb)
                self.get_logger().info(f'publish_rate changed to {new_rate} Hz')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = HMC5883LCompassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
