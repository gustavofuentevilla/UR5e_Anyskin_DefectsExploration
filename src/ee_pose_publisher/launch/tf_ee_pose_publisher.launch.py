from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ee_pose_publisher',
            executable='tf_pose_publisher',
            name='tf_pose_publisher',
            output='screen',
            parameters=[{'update_rate': 200}],
        ),
        Node(
            package='ee_pose_publisher',
            executable='ee_pose_publisher',  # This should match your entry point in setup.py
            name='ee_pose_publisher',
            output='screen',
        )
    ])