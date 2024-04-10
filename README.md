# Assignment Documentation

In this assignment i used ROS2 humble Distribution

## Q1. Documentation (bot_description)

### Launch the bot_urdf in RViz
```bash
ros2 launch bot_description rviz.launch.py
```
![plot](./prakash_ws/src/image/rviz.png)

### Launch the bot_urdf file in Gazebo

```bash
ros2 launch bot_description spawn.launch.py
```
![plot](./prakash_ws/src/image/gazebo_spawn.png)

### Run the robot inside Empty world using Teleop twist keyboard

* Step 1. Launch the model in Gazebo
```bash
. install/setup.bash && ros2 launch bot_description spawn.launch.py
```

* Step 2. Launch controller
```bash
. install/setup.bash && ros2 launch bot_description control.launch.py
```

* Step 3. Run Teleop Twist Node
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
### Output 

[![IMAGE ALT TEXT HERE](./prakash_ws/src/image/teleop_twist_keyboard_control.png)](https://github.com/Prakashdey497/ros2_custom_navigation/blob/main/prakash_ws/src/image/teleop_twist_keyboard_control.webm)



## Q2. Documentation (bot_world)

### Spawn the robot URDF in the center of the Gazebo world
```bash
. install/setup.bash && ros2 launch bot_world world.launch.py
```
### Output 
![plot](./prakash_ws/src/gazebo_world.png)



## Q3. Documentation (bot_control)

### Filter the Laser point from 0 degree to 120 degree
```bash
. install/setup.bash && ros2 run bot_control reading_laser.py
```
### Output 
![plot](./prakash_ws/src/image/laser_filter.png)



## Q4. Documentation (bot_control)

### Move the robot specified multiple goal location
```bash
. install/setup.bash && ros2 run bot_control move.py
```
### Output 
[![IMAGE ALT TEXT HERE](./prakash_ws/src/image/multiple_goal_location.png)](https://github.com/Prakashdey497/ros2_custom_navigation/blob/main/prakash_ws/src/image/multiple_goal_location.webm)


