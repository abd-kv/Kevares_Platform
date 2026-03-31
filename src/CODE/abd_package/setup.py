from setuptools import find_packages, setup

package_name = 'abd_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/follow_master_launch.py']),
        ('share/' + package_name + '/launch', ['launch/data_capture_launch.py']),
        ('share/' + package_name + '/launch', ['launch/start_zigzag_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_master_node = python_nodes.follow_master_node:main',
            'realsense_publisher_node = python_nodes.realsense_publisher_node:main',
            'robot_controller_node = python_nodes.robot_controller_node:main',
            'can_feedback_node = python_nodes.can_feedback_node:main',
            'data_capture_node = python_nodes.data_capture_node:main',
            'dodging_node = python_nodes.dodging_node:main',
            'bash_script_node = tests.bash_script_node:main',
            'can_bus_data_node = python_nodes.can_bus_data_node:main',
        ],
    },
)
