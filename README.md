# tb3
# steps
`cd to projects`
`rm -rf install build log`
`colcon build`
`source /opt/ros/foxy/setup.bash`
`source install/setup.bash`
`export TURTLEBOT3_MODEL=burger` 
## source and declare model in every terminal!
In Terminal 1 Start your physical robot's main launch file.
```
ros2 launch turtlebot3_bringup robot.launch.py
```
In Terminal 2 Launch the Gazebo world and the digital twin model.

`ros2 launch my_turtlebot3_controller launch_gazebo_twin.launch.py`


use `ros2 run my_turtlebot3_controller add_task_client --ros-args -p pickup:="station_a" -p dropoff:="station_e"` to add tasks
# manually
`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
`ros2 launch my_turtlebot3_controller my_nav2.launch.py `
then do the 2d pose estimate
`ros2 run my_turtlebot3_controller order_dispatch_node`
`ros2 run my_turtlebot3_controller mission_control_node `
`ros2 run my_turtlebot3_controller navigation_executor_node `
# single launch file
TODO
# current functionality
make delivieries between stations from a json file that contains a list
add new deliveries manually, with priority

# issues:
rviz does not always work
turtlebot stops after some time
