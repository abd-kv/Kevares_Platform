from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='abd_package',
            executable='realsense_publisher_node',
            name='realsense_publisher_node',
            output='screen'
        ),
        Node(
            package='abd_package',
            executable='can_feedback_node',
            name='can_feedback_node',
            output='screen'
        ),
        Node(
            package='abd_package',
            executable='data_capture_node',
            name='data_capture_node',
            output='screen'
        )
    ])
