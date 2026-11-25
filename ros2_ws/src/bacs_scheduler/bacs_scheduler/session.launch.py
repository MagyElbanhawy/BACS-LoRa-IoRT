import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    policy_arg = DeclareLaunchArgument('policy', default_value='fifo')
    
    # 1. The new MKR WAN 1310 Bridge
    lora_bridge_node = Node(
        package='bacs_scheduler',
        executable='lora_serial_bridge.py',
        name='mkr_lora_bridge',
        output='screen',
        parameters=[{'port': '/dev/ttyACM0'}]
    )
    
    # 2. The Scheduler (Assuming you have an executable named 'scheduler_node')
    scheduler_node = Node(
        package='bacs_scheduler',
        executable='scheduler_node',
        name='bacs_scheduler',
        output='screen',
        parameters=[{
            'policy': LaunchConfiguration('policy'),
            'trust_gate': 0.05,
            'temporal_half_life': 155.0
        }]
    )
    
    # 3. Your SLAM Node (Replace with your actual SLAM executable)
    # slam_node = Node(...)

    return LaunchDescription([
        policy_arg,
        lora_bridge_node,
        scheduler_node,
        # slam_node
    ])

