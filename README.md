# ROS Control of a Six-Freedom Robot

**Author: *Yuwei Xia***

**Date: *December 5th, 2018***

***Northwestern University***



## Abstract

The main idea of the project is to get a six-freedom robot arm to be controlled through ROS system for
robotics manipulation and further vision task with camera attached. The control board is a Pololu chip communicating with ROS laptop through USB channel.



## Overview

This is a demo showing how the robot arm is controlled with keyboard of a Linux laptop.

The video is uploaded in YouTube with the url: https://www.youtube.com/watch?v=dZh0aEDAp8o



## High Level Description

This project, at a high level, can be broken up into 3 key steps, namely **Configuration**, **USB Communication** and **Keyboard Control**.

### Configuration

This step simply involves connecting the Pololu #1350 chip to the six-freedom robot arm and a ROS laptop. Wiring the battery circuit and install the released software package to control the 6 channels through the software.

### USB Communication

This step includes creating a ROS control node to deal with the control message data type with 6 joints and other details and then set a keyboard control node for sending those data to the chip. The chip needs to receive the processed data through USB communication.

### Keyboard Control

This step encompasses the bulk of the work necessary to complete this task. It involves mainly two parts: 

1. Configure the Linux terminal settings for input and output control signals generated by keyboard.

2. Set keyboard instructions and time clocks for sending desired control signal with the normal data type.



## Step Implementation

### Configuration

1. Wire the board

   ![](/home/willx/catkin_ws/src/ros_robot/image/maestro.png)

2. Control through the Maestro Control Center. For the 6-channel board, each channel should be serving as a servo PWM output. 


### USB Communication

The Linux laptop uses /dev/ttyACM0 for Command control lines and /dev/ttyACM1 for TTL Serials. I use the command port for controlling the chip board with terminal commands. The baudrate is set as 9600 and the movement range of each joint is min 3968 to max 8000 practically. 

```python
PORT = "/dev/ttyACM0"
BAUDRATE = 9600
NUMBYTES = 4
HEADERBYTE = 0x84
MAX = 8000
MIN = 3968
```

To use the usb channel, we first need to obtain the permission from the system

```command
$ sudo chmod 666 /dev/ttyACM0
```

Since there are 6 joints to be moving of the robot, I write a message type named as JointCommand.msg for transferring the control data from ROS node to chip board via terminal. This message contains 6 float32 data.

```python
float32 joint1
float32 joint2
float32 joint3
float32 joint4
float32 joint5
float32 joint6
```

I write a JointControl class in the joint_control.py node for sending the data to the chip. The specific method is written as follow

```python
def send_command(self, channel, value):
    bfr = [0]*NUMBYTES
    bfr[0] = HEADERBYTE
    bfr[1] = channel
    target = value
    bfr[2] = target & 0x7F
    bfr[3] = (target >> 7) &0x7F
    self.comport.write(''.join(map(chr,bfr)))
    return
```
For the buffer data, we need to set its headerbyte as well as the target channel for controlled. Then to transfer the value, the specific value needs to get byte manipulation to get the desired position.

### Keyboard Control

To use laptop keyboard to control the robot, we need to use getchars() method for grabbing the instructions and send the data through terminal commands.

In the keyboard_control.py node, there is a KeyboardController class including terminal configuration, timer settings for synchronizing signals as well as publish commands.

To make the robot arm moves smoothly, the step should be set properly.

### ROS Launch

The whole project is based on a ROS project with two nodes, to start the two nodes, I write a launch file. The specific steps are listed.

```linux
$ catkin_make
$ source catkin/devel/setup.bash
$ roscore
$ roslaunch ros_robot keyboard_control.launch
```

There are 6 parameters for setting the channels respectively, and 2 nodes to be started as joint_control.py and keyboard_control.py

```python
<launch>
  <!-- ARGUMENT DEFINITIONS -->
  <arg name="joint1" default="1" doc="Channel for the joint_1 servo" />
  <arg name="joint2" default="2" doc="Channel for the joint_2 servo" />
  <arg name="joint3" default="3" doc="Channel for the joint_3 servo" />
  <arg name="joint4" default="4" doc="Channel for the joint_4 servo" />
  <arg name="joint5" default="5" doc="Channel for the joint_5 servo" />
  <arg name="joint6" default="6" doc="Channel for the joint_6 servo" />

  <!-- start the control node: -->
  <node pkg="ros_robot" type="joint_control.py" respawn="true" name="joint_controller"
		output="screen" >
	<param name="joint1_channel" value="$(arg joint1)" />
	<param name="joint2_channel" value="$(arg joint2)" />
  	<param name="joint3_channel" value="$(arg joint3)" />
	<param name="joint4_channel" value="$(arg joint4)" />
	<param name="joint5_channel" value="$(arg joint5)" />
	<param name="joint6_channel" value="$(arg joint6)" />
  </node>

  <!-- start the keyboard polling node -->
  <node pkg="ros_robot" type="keyboard_control.py" respawn="true" name="keyboard_control"
	output="screen" launch-prefix="xterm -e" />

</launch>
```



## Fallbacks and Stretch-Goals

### Fallbacks

This package could now be controlled through keyboard, but there is still some developments to be made for better performance of the robot stability. 

Meanwhile, a pre-motion planned trajectory can be traced with some modifications of the code.

### Stretch-Goals

There are mainly two fields for further study:

1. To move the end-effector to desired position with auto-generated configuration of the robot, which requires inverse kinematics packages from ROS or the MoveIt! package.
2. Use a camera to capture the image data and get processed with some computer vision algorithms for picking up specific objects.



## References

[1] Jarvis Schultz. *ME 495: Embedded Systems In Robotics*. 2018. url: http://nu-msr.github.io/embedded-course-site/

