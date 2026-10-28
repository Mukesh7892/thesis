
#!/usr/bin/env python3

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus import SMBus


MPU_ADDR = 0x68

# MPU6050 configured for +/- 2 g and +/- 250 degrees/second
ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0
GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0

# Startup gyro calibration: 150 samples at 50 Hz ~= 3 seconds
CALIBRATION_SAMPLES = 150
CALIBRATION_INTERVAL = 0.02


class IMUBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('frame_id', 'imu_link')

        bus_id = int(self.get_parameter('i2c_bus').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value

        if rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.bus = SMBus(bus_id)

        try:
            # Wake the MPU6050 from sleep.
            self.bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)

            # Gyroscope range: +/- 250 degrees/second.
            self.bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)

            # Accelerometer range: +/- 2 g.
            self.bus.write_byte_data(MPU_ADDR, 0x1C, 0x00)

            # Digital low-pass filter.
            self.bus.write_byte_data(MPU_ADDR, 0x1A, 0x03)

            # Sample-rate divider: 1000 / (1 + 19) = 50 Hz.
            self.bus.write_byte_data(MPU_ADDR, 0x19, 19)

            identity = self.bus.read_byte_data(MPU_ADDR, 0x75)
            if identity != 0x68:
                raise RuntimeError(
                    f'Unexpected MPU6050 identity: 0x{identity:02X}'
                )

            # The IMU must remain stationary during this calibration.
            self.get_logger().info(
                'Calibrating gyroscope. Keep the IMU completely still '
                'for approximately 3 seconds.'
            )
            self.gyro_offsets = self.calibrate_gyro()

        except Exception:
            self.bus.close()
            raise

        self.publisher = self.create_publisher(
            Imu, '/imu/data', 10
        )

        self.timer = self.create_timer(1.0 / rate, self.read_imu)

        self.get_logger().info(
            f'IMU bridge started: I2C bus {bus_id}, '
            f'address 0x{MPU_ADDR:02X}, rate {rate:.1f} Hz, '
            f'topic /imu/data'
        )

    def read_raw_data(self):
        """Read accelerometer and gyroscope raw values."""
        data = self.bus.read_i2c_block_data(
            MPU_ADDR, 0x3B, 14
        )

        return struct.unpack('>hhhhhhh', bytes(data))

    def calibrate_gyro(self):
        """Estimate stationary gyro bias in raw sensor counts."""
        sums = [0, 0, 0]
        successful_samples = 0

        for _ in range(CALIBRATION_SAMPLES):
            _ax, _ay, _az, _temp, gx, gy, gz = self.read_raw_data()

            sums[0] += gx
            sums[1] += gy
            sums[2] += gz
            successful_samples += 1

            time.sleep(CALIBRATION_INTERVAL)

        if successful_samples == 0:
            raise RuntimeError('Gyroscope calibration received no samples')

        offsets = tuple(
            total / successful_samples for total in sums
        )

        self.get_logger().info(
            'Gyro calibration complete. Raw offsets: '
            f'X={offsets[0]:.2f}, '
            f'Y={offsets[1]:.2f}, '
            f'Z={offsets[2]:.2f}'
        )

        return offsets

    def read_imu(self):
        try:
            ax, ay, az, _temp, gx, gy, gz = self.read_raw_data()

            # Remove the stationary gyro bias measured at startup.
            gx_corrected = gx - self.gyro_offsets[0]
            gy_corrected = gy - self.gyro_offsets[1]
            gz_corrected = gz - self.gyro_offsets[2]

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Acceleration: raw counts -> m/s^2.
            # Acceleration is not bias-corrected in this version.
            msg.linear_acceleration.x = (
                ax / ACCEL_SCALE * GRAVITY
            )
            msg.linear_acceleration.y = (
                ay / ACCEL_SCALE * GRAVITY
            )
            msg.linear_acceleration.z = (
                az / ACCEL_SCALE * GRAVITY
            )

            # Corrected angular velocity: raw counts -> rad/s.
            msg.angular_velocity.x = (
                gx_corrected / GYRO_SCALE * DEG_TO_RAD
            )
            msg.angular_velocity.y = (
                gy_corrected / GYRO_SCALE * DEG_TO_RAD
            )
            msg.angular_velocity.z = (
                gz_corrected / GYRO_SCALE * DEG_TO_RAD
            )

            # Orientation is not estimated by this driver.
            msg.orientation_covariance[0] = -1.0
            # Angular velocity covariance: (rad/s)^2
            msg.angular_velocity_covariance[0] = 0.01
            msg.angular_velocity_covariance[4] = 0.01
            msg.angular_velocity_covariance[8] = 0.0004

            # Linear acceleration covariance: (m/s^2)^2
            msg.linear_acceleration_covariance[0] = 0.04
            msg.linear_acceleration_covariance[4] = 0.04
            msg.linear_acceleration_covariance[8] = 0.04

            # Covariances still need to be measured/configured
            # before using this message in an EKF.
            self.publisher.publish(msg)

        except OSError as exc:
            self.get_logger().error(f'I2C read failed: {exc}')

    def destroy_node(self):
        if hasattr(self, 'bus'):
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = IMUBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            print(f'IMU bridge stopped: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

