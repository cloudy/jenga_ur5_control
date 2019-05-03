# jenga-ur5-control

## Purpose

MoveIt! scripts for controlling UR5 with custom solenoid-based gripper.




## Instructions

Following the usage instructions in `ros-setup.sh` located in https://github.com/jovialmcnulty/ros-setup will do the following:

- Setup ROS
- Setup catkin workspace
- Install UR5 ROS packages

After the setup is complete, the following commands can be used to test a simulated UR5:

NOTE: You will need running in a GUI session to use the simulators.

Multiple terminals will be needed, make sure to `cd ~/ros_ws && source devel/setup.${0}` for each terminal session.

First terminal: `roslaunch ur_bringup ur5_bringup.launch`

Second terminal: `roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`

Third terminal:  `roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

Use the following terminal to subscribe to various topics to see what is being published by running ROS topics.

For example: `rostopic echo /arm_controller/state`

With Rviz running, you should be able to drag the end-effector to a random position within
the UR5's workspace. Then from the 'planning' tab, you should be able to plan and execute a path
which will be played back in real-time on both Rviz and the open Gazebo session.

