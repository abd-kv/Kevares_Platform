from setuptools import find_packages, setup

package_name = 'vaughan_pilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['launch/data_capture_launch.py']),
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
            'rear_left_realsense_publisher_node = vaughan_pilot.rear_left_realsense_publisher_node:main',
            'rear_right_realsense_publisher_node = vaughan_pilot.rear_right_realsense_publisher_node:main',
            'left_realsense_publisher_node = vaughan_pilot.left_realsense_publisher_node:main',
            'right_realsense_publisher_node = vaughan_pilot.right_realsense_publisher_node:main',
            'data_capture_node = vaughan_pilot.data_capture_node:main',
            'can_feedback_node = vaughan_pilot.can_feedback_node:main',
            'mask_creation_node = vaughan_pilot.mask_creation_node:main', 
            'segmentation_node = vaughan_pilot.segmentation_node:main',
            'gps_node = vaughan_pilot.gps_node:main'
            

        ],
    },
)
