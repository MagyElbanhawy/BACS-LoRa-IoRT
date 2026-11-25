"""ROS 2 launch file for a numbered physical LoRa link test.

Brings up the scheduler node with parameters supplied by record_session.sh:

    ros2 launch bacs_scheduler session.launch.py \
        policy:=LINK_TEST \
        serial_dev:=/dev/ttyACM0 \
        serial_baud:=115200 \
        session_dir:=~/hardware_runs/HWS-LINK-001-... \
        session_id:=HWS-LINK-001-...
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('policy',       default_value='LINK_TEST'),
        DeclareLaunchArgument('config',      default_value=''),
        DeclareLaunchArgument('serial_dev',  default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('session_dir', default_value=''),
        DeclareLaunchArgument('session_id',  default_value=''),
        DeclareLaunchArgument('tx_period_s', default_value='11.0'),
        DeclareLaunchArgument('payload_bytes', default_value='52'),
        DeclareLaunchArgument('duty_cycle', default_value='0.01'),
        Node(
            package='bacs_scheduler',
            executable='scheduler_node',
            name='bacs_scheduler',
            output='screen',
            parameters=[{
                'policy':        LaunchConfiguration('policy'),
                'config':        LaunchConfiguration('config'),
                'serial_dev':    LaunchConfiguration('serial_dev'),
                'serial_baud':   ParameterValue(
                    LaunchConfiguration('serial_baud'), value_type=int
                ),
                'session_dir':   LaunchConfiguration('session_dir'),
                'session_id':    LaunchConfiguration('session_id'),
                'tx_period_s':   ParameterValue(
                    LaunchConfiguration('tx_period_s'), value_type=float
                ),
                'payload_bytes': ParameterValue(
                    LaunchConfiguration('payload_bytes'), value_type=int
                ),
                'duty_cycle':    ParameterValue(
                    LaunchConfiguration('duty_cycle'), value_type=float
                ),
            }],
        ),
    ])
