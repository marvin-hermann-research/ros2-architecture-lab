from setuptools import find_packages, setup

package_name = 'bipedal_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'py_trees_ros',
    'py_trees',
    'pyyaml',
    ],
    zip_safe=True,
    maintainer='marvin',
    maintainer_email='marvin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_sensor = bipedal_robot_pkg.ros_nodes.sensors.imu_sensor_node:main',
            'laser_sensor = bipedal_robot_pkg.ros_nodes.sensors.laser_sensor_node:main',
            'battery_monitor = bipedal_robot_pkg.ros_nodes.sensors.battery_monitor_node:main',
        ],
    }

)
