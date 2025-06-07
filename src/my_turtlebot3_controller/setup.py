from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_turtlebot3_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtlebot = my_turtlebot3_controller.move_turtlebot:main',
            'GoToBinsNode = my_turtlebot3_controller.GoToBinsNode:main',
            'cmd_vel_relay_node = my_turtlebot3_controller.cmd_vel_relay:main',
            'DecisionNode = my_turtlebot3_controller.DecisionNode:main', 
            'order_dispatch_node = my_turtlebot3_controller.order_dispatch_node:main',
            'mission_control_node = my_turtlebot3_controller.mission_control_node:main',
            'BinSensorMockNode = my_turtlebot3_controller.bin_sensor_mock_node:main',
            'navigation_executor_node = my_turtlebot3_controller.NavigationExecutorNode:main'
        ],
    },
)
