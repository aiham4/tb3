import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_controller = get_package_share_directory('lab_turtlebot3')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(pkg_my_controller, 'worlds', 'simluation_final.world')

    # Path to the new twin model we created
    sdf_file = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_burger_robot1', 'model.sdf')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_file}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
        ),
        # Spawn the robot twin. Its name and namespace are defined in the SDF.
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-file', sdf_file, '-entity', 'robot1', '-x', '-2.0', '-y', '-0.5', '-z', '0.01'],
            output='screen'
        )
    ])
