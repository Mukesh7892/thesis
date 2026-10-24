#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import socket

class WirelessBridge(Node):
    def __init__(self):
        super().__init__('wireless_bridge')

        # Declare parameters
        self.declare_parameter('esp_ip', '192.168.1.50') # Change to your ESP32 IP
        self.declare_parameter('esp_port', 8888)

        self.esp_ip = self.get_parameter('esp_ip').value
        self.esp_port = self.get_parameter('esp_port').value

        # UDP Socket configuration
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.get_logger().info(f"Targeting ESP32 at {self.esp_ip}:{self.esp_port}")

        # Subscribers and Publishers
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pub_feedback = self.create_publisher(
            Int32MultiArray,
            '/wheel_rpm',
            10
        )

        # 20 Hz timer to read incoming telemetry UDP packets
        self.create_timer(0.05, self.read_udp_feedback)

    def cmd_vel_callback(self, msg: Twist):
        cmd_str = f"V {msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        try:
            self.sock.sendto(cmd_str.encode('utf-8'), (self.esp_ip, self.esp_port))
        except Exception as e:
            self.get_logger().warn(f"Failed to send UDP packet: {e}")

    def read_udp_feedback(self):
        try:
            while True:
                data, _ = self.sock.recvfrom(256)
                line = data.decode('utf-8').strip()
                if line.startswith("FEEDBACK"):
                    parts = line.split()
                    if len(parts) == 3:
                        msg = Int32MultiArray()
                        msg.data = [int(parts[1]), int(parts[2])]
                        self.pub_feedback.publish(msg)
        except BlockingIOError:
            pass # No packet available
        except Exception as e:
            self.get_logger().error(f"Error reading socket: {e}")

    def destroy_node(self):
        # Emergency stop on shutdown
        try:
            self.sock.sendto(b"V 0.0 0.0\n", (self.esp_ip, self.esp_port))
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WirelessBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()