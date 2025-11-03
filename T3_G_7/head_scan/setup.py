from setuptools import find_packages, setup

package_name = 'head_scan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loreg',
    maintainer_email='loreg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_movement_client = head_scan.head_movement_client:main',
            'aruco_scan_publisher = head_scan.aruco_scan_publisher:main',
            'aruco_coord_transformation = head_scan.aruco_coord_transformation:main',
            'state_machine_node = head_scan.state_machine_node:main',
            'motion_planner_node = head_scan.motion_planner_node:main',
            'lbd_movement_node = head_scan.lbd_movement_node:main',
            'lbd_state_machine = head_scan.lbd_state_machine:main',
            'lbd_motion_planner = head_scan.lbd_motion_planner:main'
        ],
    },
)
