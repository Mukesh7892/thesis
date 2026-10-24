
#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


class MotorBridgeTest(Node):
    def __init__(self):
        super().__init__('motor_bridge_test')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('command_timeout', 0.30)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )

        self.last_cmd_time = time.monotonic()
        self.have_command = False
        self.timeout_stop_sent = False
        self.last_reported_status = None

        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=0.05
            )

            # Opening a serial port may reset some ESP32 boards.
            time.sleep(2.0)
            self.ser.reset_input_buffer()

            self.get_logger().info(
                f'Connected to ESP32 on {self.port}'
            )

        except (serial.SerialException, OSError) as exc:
            raise RuntimeError(
                f'Cannot open serial port {self.port}: {exc}'
            ) from exc

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Keep serial reading and watchdog checks separate from callbacks.
        self.timer = self.create_timer(0.02, self.update)

        self.send_stop()
        self.get_logger().info(
            'Ready. Waiting for /cmd_vel. '
            'This node does not publish odometry.'
        )

    def send_line(self, message: str):
        try:
            self.ser.write((message + '\n').encode('ascii'))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(
                f'Serial write failed: {exc}'
            )

    def send_stop(self):
        self.send_line('V 0.0 0.0')

    def cmd_callback(self, msg: Twist):
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self.get_logger().error(
                'Rejected non-finite velocity command'
            )
            self.send_stop()
            return

        # Firmware independently enforces its own speed limits.
        packet = f'V {linear_x:.4f} {angular_z:.4f}'
        self.send_line(packet)

        self.last_cmd_time = time.monotonic()
        self.have_command = True
        self.timeout_stop_sent = False

    def update(self):
        # Read complete response lines without blocking.
        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()

                if not raw:
                    break

                line = raw.decode(
                    'utf-8', errors='replace'
                ).strip()

                if not line:
                    continue

                if line.startswith('ERR'):
                    self.get_logger().error(
                        f'ESP32: {line}'
                    )
                elif line.startswith('READY'):
                    self.get_logger().info(
                        'ESP32 firmware is ready'
                    )
                elif line.startswith('STOP'):
                    self.get_logger().warning(
                        f'ESP32: {line}'
                    )
                elif line.startswith('STATUS'):
                    if line != self.last_reported_status:
                        self.last_reported_status = line
                        self.get_logger().info(
                            f'ESP32 {line}'
                        )
                # CMD responses are intentionally ignored to avoid
                # flooding the ROS log during teleoperation.

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(
                f'Serial read failed: {exc}'
            )
            return

        # The bridge stops when its command source becomes stale.
        # The ESP32 has its own independent watchdog as a backup.
        age = time.monotonic() - self.last_cmd_time

        if (
            self.have_command
            and age > self.command_timeout
            and not self.timeout_stop_sent
        ):
            self.send_stop()
            self.timeout_stop_sent = True

            self.get_logger().warning(
                'Command timeout: sent zero velocity'
            )

    def destroy_node(self):
        try:
            # Send several zero commands during orderly shutdown.
            if hasattr(self, 'ser') and self.ser.is_open:
                for _ in range(3):
                    self.send_stop()
                    time.sleep(0.02)

                self.ser.close()
        except Exception as exc:
            self.get_logger().warning(
                f'Shutdown stop attempt failed: {exc}'
            )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MotorBridgeTest()
        rclpy.spin(node)

    except (KeyboardInterrupt, RuntimeError):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

