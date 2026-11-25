"""ROS 2 launch file for a BACS scheduler session with real constraints.

Brings up the BACS scheduler node (scheduler_node_bacs) with parameters
supplied by record_bacs_sessions.sh:

    ros2 launch bacs_scheduler session_bacs.launch.py \
        policy:=FIFO \
        serial_dev:=/tmp/pty0 \
        serial_baud:=115200 \
        session_dir:=~/hardware_runs/HWS-BACS-FIFO-R1-... \
        session_id:=HWS-BACS-FIFO-R1-... \
        tx_period_s:=11.0 \
        constraint_seed:=42 \
        constraint_period_s:=2.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('policy',              default_value='FIFO'),
        DeclareLaunchArgument('config',             default_value=''),
        DeclareLaunchArgument('serial_dev',         default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud',        default_value='115200'),
        DeclareLaunchArgument('session_dir',        default_value=''),
        DeclareLaunchArgument('session_id',         default_value=''),
        DeclareLaunchArgument('tx_period_s',        default_value='11.0'),
        DeclareLaunchArgument('payload_bytes',      default_value='52'),
        DeclareLaunchArgument('constraint_seed',    default_value='42'),
        DeclareLaunchArgument('constraint_period_s', default_value='2.0'),
        DeclareLaunchArgument('trust_threshold',    default_value='0.5'),
        Node(
            package='bacs_scheduler',
            executable='scheduler_node_bacs',
            name='bacs_scheduler',
            output='screen',
            parameters=[{
                'policy':               LaunchConfiguration('policy'),
                'config':               LaunchConfiguration('config'),
                'serial_dev':           LaunchConfiguration('serial_dev'),
                'serial_baud':          LaunchConfiguration('serial_baud'),
                'session_dir':          LaunchConfiguration('session_dir'),
                'session_id':           LaunchConfiguration('session_id'),
                'tx_period_s':          LaunchConfiguration('tx_period_s'),
                'payload_bytes':        LaunchConfiguration('payload_bytes'),
                'constraint_seed':      LaunchConfiguration('constraint_seed'),
                'constraint_period_s':  LaunchConfiguration('constraint_period_s'),
                'trust_threshold':     LaunchConfiguration('trust_threshold'),
            }],
        ),
    ])
