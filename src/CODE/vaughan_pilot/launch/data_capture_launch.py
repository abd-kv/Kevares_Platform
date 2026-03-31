from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vaughan_pilot',
            executable='rear_left_realsense_publisher_node',
            name='rear_left_realsense_publisher_node',
            output='screen'
        ),
        Node(
            package='vaughan_pilot',
            executable='rear_right_realsense_publisher_node',
            name='rear_right_realsense_publisher_node',
            output='screen'
        ),
        Node(
            package='vaughan_pilot',
            executable='can_feedback_node',
            name='can_feedback_node',
            output='screen'
        ),
        Node(
            package='vaughan_pilot',
            executable='data_capture_node',
            name='data_capture_node',
            output='screen'
        # ),
        # Node(
        #     package='vaughan_pilot',
        #     executable='mask_creation_node',  # Added mask_creation_node
        #     name='mask_creation_node',
        #     output='screen'
        # ),
        # Node(
        #     package='vaughan_pilot',
        #     executable='segmentation_node',  # Added mask_creation_node
        #     name='segmentation_node',
        #     output='screen'
        ),
        Node(
            package='vaughan_pilot',
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),
    ])

