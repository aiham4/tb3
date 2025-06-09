import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_controller = get_package_share_directory('my_turtlebot3_controller')

    # We need a different RViz config for SLAM, or we can just use the default.
    # For now, let's use the one we have, but you may need to adjust it.
    rviz_config_file = os.path.join(pkg_my_controller, 'config', 'nav_view.rviz')

    return LaunchDescription([
        # Start the SLAM ToolBox node (in synchronous mode)
        Node(
            parameters=[{'use_sim_time': False}], # SLAM runs on real-world time
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        ),
        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
