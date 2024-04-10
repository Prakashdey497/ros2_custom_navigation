# Assignment Documentation

In this assignment i used ROS2 humble Distribution

## Q1. Documentation (bot_description)

### Launch the bot_urdf in rviz
```bash
ros2 launch bot_description rviz.launch.py
```
![plot](./image/rviz.png)

### Launch the bot_urdf file in Gazebo
```bash
ros2 launch bot_description spawn.launch.py
```
![plot](./image/gazebo_spawn.png)

### Run the robot inside Empty world using Teleop twist keyboard 

* Step1. Launch the model in gazebo
```bash
. install/setup.bash && ros2 launch bot_description spawn.launch.py
```

* Step2. Launch controller
```bash
. install/setup.bash && ros2 launch bot_description control.launch.py
```

* Step3. Run Teleop Twist Node
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
### Output 

[](./image/teleop_twist_keyboard_control.webm)



## Q2. Documentation (bot_world)

### Spawn the robot urdf in the centre of the gazebo world
```bash
. install/setup.bash && ros2 launch bot_world world.launch.py
```
### Output 
![plot](./image/gazebo_world.png)

## Q3. Documentation (bot_control)

### Filter the Laser point from 0 degree to 120 degree
```bash
. install/setup.bash && ros2 run bot_control reading_laser.py
```
### Output 
![plot](./image/lase_filter.png)

## Q4. Documentation (bot_control)

### Move the robot specified multiple goal location
```bash
. install/setup.bash && ros2 run bot_control move.py
```
### Output 
![plot](./image/multiple_goal_location.webm)

