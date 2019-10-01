# MOVR ROS Workspace
> MOVR - Autonomous vehicle for close proximity navigation

## Table of contents

- [Workspace packages](#workspace-packages)
- [Dependencies](#dependencies)
  - [ROS](#ros)
  - [Python](#python)
- [Installation](#installation)
- [How to use](#how-to-use)
- [Development](#development)
- [References](#references)

## Workspace packages

### movr_teleop
Package that contains scripts to teleoperate via joystick, keyboard and speech

### movr_vision
Package that contains scripts related to vision sensors.

## Dependencies

### ROS 
#### rosserial
```
# rosserial
$ sudo apt install ros-kinetic-rosserial
$ sudo apt install ros-kinetic-rosserial-arduino 
$ sudo apt install ros-kinetic-rosserial-python
```
### ackermann-msgs
```
# ackermann-msgs
$ sudo apt install ros-kinetic-ackermann-msgs
```

#### joy
```
# Joy
$ sudo apt install ros-kinetic-joy

# Set permission to make accessible to ROS
$ ls -l /dev/input/jsX              # X is the joystick number
$ sudo chmod a+rw /dev/input/jsX
```
#### openni
```
# Openni - Kinect
$ sudo apt install ros-kinetic-openni-launch
```
#### opencv 3
```
# Kinect OpenCV 3
$ sudo apt install ros-kinetic-opencv3
```
#### cv-bridge
```
# CV-Bridge
$ sudo apt install ros-kinetic-cv-bridge
# NOTE: You will need to go into the package's CMakeLists.txt and add the following line:
find_package(OpenCV REQUIRED)
# Do not use find_package(catkin ... but add another statement find_package(OpenCV REQUIRED)
```

#### astra
```
# Install Orbbec Astra camera dependencies
$ sudo apt install ros-kinetic-astra-camera
$ sudo apt install ros-kinetic-astra-launch
```
Follow configuration instructions outlined [here](http://wiki.ros.org/astra_camera).

### Python
#### API AI
```
# API.AI
# Chat bot API
# https://github.com/api-ai/apiai-python-client
$ pip install apiai
```
#### SpeechRecognition
```
# SpeechRecognition
# Google speech recognition API wrapper
# https://pypi.python.org/pypi/SpeechRecognition/
$ pip install SpeechRecognition
```
#### gTTS
```
# gTTS 1.2.1
# Google text-to-speech API wrapper
# https://pypi.python.org/pypi/gTTS/1.2.1
$ pip install gTTS
```
#### pocketsphinx-python
```
# pocketsphinx-python
# CMU speech recognition
# https://github.com/cmusphinx/pocketsphinx-python
# sudo apt-get install python python-all-dev python-pip build-essential swig git libpulse-dev
$ pip install pocketsphinx
```
#### pygame
```
# pygame
# Game engine. Required to use audio library
# https://www.pygame.org/wiki/GettingStarted
$ pip install pygame
```

#### playsound
```
# playsound
# Pure Python, cross platform, single function module with no dependencies for playing sounds.
# https://pypi.org/project/playsound/
$ pip install playsound
```

#### keyboard
```
# keyboard
# Take full control of your keyboard with this small Python library. 
# Hook global events, register hotkeys, simulate key presses and much more.
# https://pypi.org/project/keyboard/
# pip install keyboard
```

#### json
```
# json
# JSON library
$ pip install json
```

## Installation

```
# Go to top-level worspace directory
$ cd movr_ws/
```

```
# Make workspace
# This generates the 'build' and 'devel' directories
# The 'devel' directory has the setup directories and setup files
$ catkin_make
```

```
# Go into 'movr_ws' directory
$ cd movr_ws/ 

# Source the workspace
$ source devel/setup.bash
```

## How to use

### In Workstation
```
# Run roscore
$ roscore
```

```
# Run ROS Joy node. Opens joystick device and publishes input
$ rosrun joy joy_node
```

```
# Run movr joy node
$ rosrun movr_teleop joyjoy.py
```
### In MOVR Vehicle
```
# Run Python rosserial node
$ rosrun rosserial_python serial_node.py /dev/ttyACM1   # Check device name 'ls -la /dev'
```

```
# Run voice interface
$ rosrun movr_voice hello_movr.py
```
### View Astra Camera
```
# Launch Astra camera node
$ roslaunch astra_launch astra.launch
```

```
# View RGB image
$ rosrun image_view image_view image:=/camera/rgb/image_raw
```

```
# View Depth image
$ rosrun image_view image_view image:=/camera/depth/image
```
### View Kinect Sensor

```
# Launch openni
# Execute this first before running any of the commands below
$ roslaunch openni_launch openni.launch
```

```
# View RGB image
$ rosrun image_view image_view image:=/camera/rgb/image_color
```

```
# View depth image
$ rosrun image_view image_view image:=/camera/depth/image
```

```
# View depth-disparity
$ rosrun image_view disparity_view image:=/camera/depth/disparity
```
#### In RViz
```
# To view RGB - depth correlated images, execute the following
# 1. Remap Depth and Correlate with RGB
$ rosrun rqt_reconfigure rqt_reconfigure
# Open the 'driver' option and select 'reconfigure
#
# 2. Run rviz
$ rosrun rviz rviz
# In cloud_points 2, switch the topic to:  '/camera/depth_registered/points'
```

## Development

## References
- [ROS Ackermann Interest Group](http://wiki.ros.org/Ackermann%20Group)
- [ROS Joy](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
- [ROS Openni Launch](http://wiki.ros.org/openni_launch/Tutorials/QuickStart)
- [ROS Time Elastic Band Planning](http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots)
