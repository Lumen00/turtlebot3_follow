# 41014 Sensors and Control Project 2: Turtlebot Following a Person

## Dependencies
- Ubuntu 20.04.6 
- ROS Noetic
- OpenCV
- Point Cloud Library
- RealSense2 ROS 

## Installation
### ROS Noetic
For detailed instructions to install ROS Noetic on Ubuntu, follow this [link](http://wiki.ros.org/noetic/Installation/Ubuntu).

ROS Noetic will be used to manage the communication between nodes and with the turtlebot.

### OpenCV
OpenCV requires installation for C++ to be done by building from source. For detailed instructions to install OpenCV on Ubuntu, follow this [link](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html).

OpenCV will be used to enable the tracker functionality in the repository.

### Point Cloud Library (PCL)
Installation of PCL for Linux users can be done simply via a package manager.

```
sudo apt install libpcl-dev
```
For instructions to install PCL through other methods, follow this [link](https://pointclouds.org/downloads/).

### Realsense2 ROS
Follow installation instructions from [this](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy) git repo for ROS Noetic.

## Usage
This repository can be used for both simulation of a waffle turtlebot following a dummy person and with a real robot with an RGB-D camera following a real person.

Clone this repository into your `catkin_ws/src` directory.
```
git clone https://github.com/Lumen00/turtlebot3_follow
```

Before continuing, ensure that your ros package is built by navigating into `catkin_ws` and running `catkin_make`.

You are now ready to run either the simulation or real robot operations.

### Running w/ Simulation
Navigate to the directory `catkin_ws/src/turtlebot3_follow/src/follow_sim` and open the code in vscode (or your editor of choice). From line 386 onwards, comment and uncomment lines which specify if they are for simulation or the real robot.

This section of the guide will require you to open four terminals. Run the following commands in order in each terminal. Note that the virtual turtlebot model used here is the waffle, which comes with an in-built RGB-D camera.
1. `roscore`
2. 
```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_follow turtlebot3_person.launch
```
3. 
```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
4. `rosrun turtlebot3_follow follow_sim`

The commands from the second terminal will open a Gazebo test environment with a turtlebot and a virtual model of a person.

### Running w/ Real Robot
Follow this [online guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to set up IP addresses for your turtlebot and your host machine. You must be connected to the same local network.

Navigate to the directory `catkin_ws/src/turtlebot3_follow/src/follow_sim` and open the code in vscode (or your editor of choice). From line 386 onwards, comment and uncomment lines which specify if they are for simulation or the real robot.

This section of the guide will require you to open five terminals. Run the following commands in order in each terminal. Note that the `waffle_pi` turtlebot model is being used, which does not come with an RGB-D camera by default. This project uses an Intel Realsense D435 RGB-D camera which is plugged into the host computer and mounted onto the turtlebot. If you are able to configure your turtlebot to run a connected RGB-D camera, you should do so.
1. `roscore`
2. 
```
ssh ubuntu@[turtlebot's ip address]
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
3. `roslaunch realsense2_camera rs_rgbd.launch`
4. 
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
5. `rosrun turtlebot3_follow follow_sim`

### Selecting a feature to track
Once all terminals have had their commands processed, you will see a window showing the current RGB image of the camera. Select a region of interest by clicking with the mouse cursor and dragging over the desired feature. Press c to cancel the selection and refresh the RGB image shown. If the selection is satisfactory, press either `space` or `enter`. A new window will open with a bounding box drawn over the feature while the turtlebot navigates to follow it.

![tracking window](https://github.com/Lumen00/turtlebot3_follow/blob/master/images/tracker%20with%20bbox.png)

The turtlebot will attempt to keep the feature within 2m of the camera and in the middle of the recorded image.

To stop the following routine, use `ctrl+c` in the terminal which you ran the `follow_sim` command.


