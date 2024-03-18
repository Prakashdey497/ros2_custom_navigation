# Documentation Of the ros2 Turtlebot3 waffle Mapping and Navigation

## Installation Package

### 1. Install Turtlebot3 package

```bash
sudo apt install ros-humble-turtlebot3*
```

### 2. Install the Nav2 packages

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Make the robot move in the environment

1. Add this line at the end of your `.bashrc`: 
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```
   After adding this line, make sure to source the `.bashrc` or open a new terminal.

2. Start the simulated robot in a Gazebo world. (Note: It may take some time on the first launch)
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
   ```

3. Now that we have the robot inside a room, let’s make it move. In a new terminal, start the teleop node.
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

## Generate a map with ROS2 Nav2 using SLAM (Two way we can generate Map)

## Method1.

### Start SLAM functionality and RViz

First, ensure you have started the robot, here, Turtlebot3 in Gazebo.
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

1. Start the SLAM functionality for Turtlebot3.
   ```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
   ```
   Use the `"use_sim_time"` argument because we are running on Gazebo.

2. Now, make the robot move in the world. Start the Turtlebot3 teleop node.
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

3. To save the map, open a new terminal and run this command.
   ```bash
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

## Method2.

First, ensure you have started the robot, here, Turtlebot3 in Gazebo.

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
0. build And Source The workspace

   ```bash
   colcon build && . install/setup.bash
   ```
1. Start the SLAM functionality for Turtlebot3.
   ```bash
   ros2 launch turtlebot_waffle_mapping online_async_launch.launch.py
   ```
   Use the `"use_sim_time"` argument because we are running on Gazebo.

2. Now, make the robot move in the world. Start the Turtlebot3 teleop node.
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

3. To save the map, open a new terminal and run this command.
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ${path/where/you/want/to/save/map_name}
   ```

## Quick fix and DDS issue with Nav2

Before we continue with Navigation, we need to fix 2 small things.

First, as you may know, ROS2 communication is based on DDS (for the middleware). No need to dive into this now, you just have to know that there are several possible DDS implementations, and the default one for ROS2 is Fast DDS. Unfortunately it doesn’t work so well with Nav2, so it’s been recommended to use Cyclone DDS instead.

### Install and setup Cyclone DDS for ROS2. 
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Add this line at the end of your `.bashrc`: 
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
After adding this line, make sure to source the `.bashrc` or open a new terminal.

One more thing to do, and this is a Turtlebot3 specific issue.

Open this param file:
```bash
sudo gedit /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml
```
Find this line, and replace it like you see below.

```bash
#robot_model_type: "differential"
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

## Start Navigation 2 for the robot

Start from a clean environment – stop everything, close and reopen all terminals.

Now, start the robot again.
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

In another terminal, start the Navigation stack, and provide the map as an argument.
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=path/to/my_map.yaml
```

### 2D pose estimate and Navigation goals

Click on the ``2D pose estimate`` button on RViz. Then click on the map where the robot is (you should see that on Gazebo). Maintain the click to also specify the orientation with a green arrow.

Finally, you can give navigation commands! Click on ``Nav2 Goal``, then click on the map to select a position + orientation, and the robot should start to go to that pose. You can verify on Gazebo, to see that the robot is actually moving.