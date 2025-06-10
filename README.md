# tb3
# steps
cd to workspace
```
cd tb3
source /opt/ros/foxy/setup.bash
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```
## source and declare model in every terminal!
In Terminal 1 Start your physical robot's main launch file.
```
ros2 launch turtlebot3_bringup robot.launch.py
```
In Terminal 2 Launch the Gazebo world and the digital twin model.

`ros2 launch lab_turtlebot3 launch_gazebo_twin.launch.py`

In terminal 3 launch rviz and nav2 (replace path and 2d pose estimate)

`ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$(ros2 pkg prefix lab_turtlebot3)/share/lab_turtlebot3/config/map.yaml`

Mimic navigation node

`ros2 run lab_turtlebot3 cmd_vel_relay_node`

Test possible navigation + position node

`ros2 run lab_turtlebot3 tf_to_gazebo_node`

Run functional nodes

```
ros2 run lab_turtlebot3 navigation_executor_node

ros2 run lab_turtlebot3 mission_control_node

ros2 run lab_turtlebot3 battery_simulator_node
```

use `ros2 run lab_turtlebot3 add_task_client --ros-args -p pickup:="station_a" -p dropoff:="station_e"` to add tasks


# single launch file
TODO
# current functionality
make delivieries between stations from a json file that contains a list
add new deliveries manually, with priority

# issues:
Possible issues with Namespaces, physical robot and navigation?
