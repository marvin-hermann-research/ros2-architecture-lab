from setuptools import find_packages, setup

package_name = 'bipedal_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # add package data for behaviour tree patterns
    package_data={
        package_name: [
            'behaviour_tree/patterns/*.yaml',
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # copy all YAML files into the share directory
        ('share/' + package_name + '/behaviour_tree/patterns',
            [
                'bipedal_robot_pkg/behaviour_tree/patterns/all_patterns.yaml',
                'bipedal_robot_pkg/behaviour_tree/patterns/idle_pattern.yaml',
                'bipedal_robot_pkg/behaviour_tree/patterns/walk_forward_pattern.yaml',
            ]
        )
    ],
    install_requires=[
    'setuptools>=65',
    'py_trees_ros',
    'py_trees',
    'pyyaml',
    ],
    zip_safe=True,
    maintainer='Marvin',
    maintainer_email='marvinmail06@gmail.com',
    description='ROS2 package implementing a bipedal robot behaviour tree framework for research and simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_sensor = bipedal_robot_pkg.ros_nodes.sensors.imu_sensor_node:main',
            'laser_sensor = bipedal_robot_pkg.ros_nodes.sensors.laser_sensor_node:main',
            'battery_monitor = bipedal_robot_pkg.ros_nodes.sensors.battery_monitor_node:main',
            'robot_application = bipedal_robot_pkg.behaviour_tree.launch.bipedal_robot_application:main',
        ],
    }

)
