#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import math
import time

class Nav2SerialBridge(Node):
    def __init__(self):
        super().__init__('nav2_serial_bridge')

        # Configuration parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.0575)  # meters (DDSM115)
        self.declare_parameter('track_width', 0.30)    # meters (wheelbase width)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.r = self.get_parameter('wheel_radius').value
        self.b = self.get_parameter('track_width').value

        # Connect to Serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            time.sleep(2.0)  # Wait for ESP32 boot
            self.get_logger().info(f"Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open {self.port}: {e}")
            exit(1)

        # Odometry State Variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # ROS 2 Pub/Sub
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # High-frequency serial polling & odom calculation (30 Hz)
        self.create_timer(0.033, self.update_loop)

    def cmd_vel_callback(self, msg: Twist):
        # Transmit: "V <lin_x> <ang_z>\n"
        packet = f"V {msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        try:
            self.ser.write(packet.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def update_loop(self):
        # 1. Read Serial Buffer
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("FB"):
                    parts = line.split()
                    if len(parts) == 3:
                        rpm_l = float(parts[1])
                        rpm_r = float(parts[2])
                        self.compute_odometry(rpm_l, rpm_r)
            except Exception:
                pass

    def compute_odometry(self, rpm_l, rpm_r):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0.0 or dt > 1.0:
            self.last_time = current_time
            return

        self.last_time = current_time

        # Convert RPM to linear velocity (m/s)
        v_l = (rpm_l * 2.0 * math.pi * self.r) / 60.0
        v_r = (rpm_r * 2.0 * math.pi * self.r) / 60.0

        # Robot forward kinematics
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.b

        # Integrate pose
        delta_x = (v * math.cos(self.th)) * dt
        delta_y = (v * math.sin(self.th)) * dt
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Euler to Quaternion for 2D Yaw
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # 1. Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # 2. Publish nav_msgs/Odometry on /odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

    def destroy_node(self):
        try:
            self.ser.write(b"V 0.0 0.0\n")
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Nav2SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()