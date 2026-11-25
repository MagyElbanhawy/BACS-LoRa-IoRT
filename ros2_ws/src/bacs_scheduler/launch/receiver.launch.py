"""ROS 2 launch file for the BACS receiver node (Raspberry Pi side).

Brings up the receiver node that reads incoming LoRa packets from the
Arduino and publishes them to /bacs/received_constraint.

    ros2 launch bacs_scheduler receiver.launch.py \
        serial_dev:=/dev/ttyACM0 \
        serial_baud:=115200 \
        session_dir:=~/hardware_runs/HWS-RX-001 \
        session_id:=HWS-RX-001
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('serial_dev',  default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('session_dir', default_value=''),
        DeclareLaunchArgument('session_id',  default_value=''),
        Node(
            package='bacs_scheduler',
            executable='bacs_receiver_node',
            name='bacs_receiver',
            output='screen',
            parameters=[{
                'serial_dev':  LaunchConfiguration('serial_dev'),
                'serial_baud': LaunchConfiguration('serial_baud'),
                'session_dir': LaunchConfiguration('session_dir'),
                'session_id':  LaunchConfiguration('session_id'),
            }],
        ),
    ])
