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
            executable='follow_master_node',
            name='follow_master_node',
            output='screen'
        ),
        Node(
            package='abd_package',
            executable='robot_controller_node',
            name='robot_controller_node',
            output='screen'
        ),
        Node(
            package='abd_package',
            executable='dodging_node',
            name='dodging_node',
            output='screen'
        ),
        # Node(
        #     package='vaughan_pilot',
        #     executable='right_realsense_publisher_node',
        #     name='right_realsense_publisher_node',
        #     output='screen'
        # ),
        # Node(
        #     package='vaughan_pilot',
        #     executable='left_realsense_publisher_node',
        #     name='left_realsense_publisher_node',
        #     output='screen'
        # )
        
    ])

