import os
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    lidar_share = get_package_share_directory('ydlidar_ros2_driver')
    rf2o_share = get_package_share_directory('rf2o_laser_odometry')
        # Robot description from Xacro
    robot_bringup_share = get_package_share_directory('robot_bringup')

    xacro_file = os.path.join(
        robot_bringup_share,
        'urdf',
        'thesis_bot1.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
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
    imu_bridge = Node(
        package='mypypkg',
        executable='imu_bridge',
        name='imu_bridge',
        output='screen',
        parameters=[{
            'i2c_bus': 7,
            'publish_rate': 50.0,
            'frame_id': 'imu_link',
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'config',
                'ekf.yaml',
            )
        ],
    )

    return LaunchDescription([
        lidar_launch,
        motor_bridge,
        imu_bridge,
        robot_state_publisher,
        rf2o_launch,
        ekf_node,
    ])
