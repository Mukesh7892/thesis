import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    lidar_share = get_package_share_directory('ydlidar_ros2_driver')
    rf2o_share = get_package_share_directory('rf2o_laser_odometry')

    # Use the configuration specifically for the YDLIDAR X2.
    lidar_params = os.path.join(
        lidar_share, 'params', 'X2.yaml'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'ydlidar_launch.py')
        ),
        launch_arguments={
            'params_file': lidar_params,
        }.items(),
    )

    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rf2o_share,
                'launch',
                'rf2o_laser_odometry.launch.py',
            )
        )
    )

    motor_bridge = Node(
        package='mypypkg',
        executable='motor_bridge',
        name='motor_bridge_test',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58A6071655-if00',
            'baud': 115200,
            'command_timeout': 0.30,
        }],
    )

    return LaunchDescription([
        lidar_launch,
        motor_bridge,
        rf2o_launch,
    ])
