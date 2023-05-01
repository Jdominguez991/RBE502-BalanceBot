# # RBE502-BalanceBot
This GitHub repository contains the source code for the course project for RBE 502: Robot Control.

The project objective was to develop different controllers to balance the Teeterbot robot. The Teeterbot robot is a simulation model created by robustify. A link to the Teeterbot repository can be found [here](https://github.com/robustify/teeterbot).

Additional modifications were added to the base package created by robustify and can be found in this repository. 

For this project, a total of 5 controllers were implemented in an attempt to balance the robot: 3 controllers using traditional approaches and 2 controllers using a state space controller. The list of controllers is described below:
1. PD Controller
2. PD with Feedforward Controller
3. PID Controller
4. PD with Feedforward Controller using State Space
5. PD with Computed Torque Controller using State Space

## Dependencies
- Make sure your system is up to date:
```
sudo apt-get update && apt-get upgrade
```
- Since the Teeterbot simulation runs on ROS 1 and Gazebo, ensure the latest distribution of ROS 1 is installed and Gazebo 9 or higher is installed.
- For keyboard control, install the teleop_twist_keyboard package
```
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

## Instructions

1. Create and initialize a catkin workspace `catkin_ws`
```
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws/src  
catkin_init_workspace  
```

2. Clone this repo to `catkin_ws/src`
```
git clone https://github.com/Jdominguez991/RBE502-BalanceBot.git
```

3. Build and source the project
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
4. To launch the simulation, different parameters can be used. The base launch command is:
```
roslaunch teeterbot_gazebo teeterbot_terrain_world.launch
```
The obstacle course command is:
```
roslaunch teeterbot_gazebo obstacle_course.launch
```
To use a different controller, append one of the following lines to the launch command.
```
controller := 0 // No Controller
controller := 1 // PD Controller (Default)
controller := 2 // PD with Feedforward Controller
controller := 3 // PID Controller
controller := 4 // PD with Feedforward in State Space
controller := 5 // PD with Computed Torque in State Space
```
To open rqt plots along with the simulation for live plotting, append one of the following lines to the launch command.
```
use_rqt := false // Default
use_rqt := true 
```
To record the robot pitch during the simulation, append one of the following lines to the launch command.
```
record_bag := true  // Default
record_bag := false 
```
To open the keyboard controller, from a different terminal, run the following command:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
If launching the file with control mode 4 or 5 and the model does not spawn correctly, launch the model with the default values at least once and then select control mode 4 or 5 to resolve the error.
