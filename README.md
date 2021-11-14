# ENPM808X Beginner Tutorials Publisher/Subscriber
<a href='https://github.com/dngo13/beginner_tutorials/blob/main/LICENSE'><img src='https://img.shields.io/badge/License-BSD_2--Clause-orange.svg'/></a>

### Diane Ngo
### ENPM808X Fall 2021

## Overview
This is a ROS Package for the beginner tutorials with creating a basic publisher and subscriber in C++. 
Reference: [link](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

## Dependencies
Must have the following installed:
- Ubuntu 18.04
- ROS Melodic
- C++, roscpp
- Standard library
- rqt_console 
- xterm (runs the subscriber in another terminal)

### Installing rqt console
```
sudo apt-get install ros-melodic-rqt ros-melodic-rqt-common-plugins
```
### Installing xterm
```
sudo apt install xterm
```
## Building Instructions
### If there is no catkin workspace
Create catkin workspace. Go to terminal and follow these commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Then source the setup.bash file with this command:
```
source devel/setup.bash
```
Now please clone the beginner_tutorials package by doing the following:
```
cd src/
git clone https://github.com/dngo13/beginner_tutorials.git
cd ..
catkin_make
source devel/setup.bash
```
### Cloning into an existing catkin workspace
Go to the src directory for the catkin workspace. And then follow these steps
```
cd ~/catkin_ws/src/
git clone https://github.com/dngo13/beginner_tutorials.git
cd ..
catkin_make
source devel/setup.bash
```
## Running the Program
Now that the package is built, we can run the program. 
In this terminal, run ```roscore```.
### Running it individually with the nodes
Open a new terminal, source it with:
```
source devel/setup.bash
```
And then run:
```
rosrun beginner_tutorials talker
```

Now, open a new terminal again, source it with:
```
source devel/setup.bash
```
And then run:
```
rosrun beginner_tutorials listener
```
You should see output in both terminals similar to ENPM808X ROS count: # and I heard: [ENPM808X ROS count: #]

### Running the launch file
Open a new terminal, source it with:
```
source devel/setup.bash
```
And then run:
```
roslaunch beginner_tutorials pub_sub_launch.launch
```
Alternatively, to change the frequency:
```
roslaunch beginner_tutorials pub_sub_launch.launch frequency:=##
```
Where ## is a number of your choice. Note that a 0 or a negative number will throw a fatal error and terminate the program.
A frequency greater than 25 will give a warning.

### Changing the string output
To call the service first check if it is seen below:
```
rosservice list
```
You should see something like: 
```
/ChangeStringOutput
/listener_node/get_loggers
/listener_node/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/rqt_gui_py_node_11776/get_loggers
/rqt_gui_py_node_11776/set_logger_level
/rqt_gui_py_node_11824/get_loggers
/rqt_gui_py_node_11824/set_logger_level
/talker_node/get_loggers
/talker_node/set_logger_level
```
Then, run the command below to change the text, with anything between the quotes as your desired output.
```
rosservice call /ChangeStringOutput "New Message" 
```

### View rosbag file 
There is a rosbag file already recorded in the /results directory. To view it, do the following commands in a terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/beginner_tutorials/results
rosbag info rosbag_record.bag 
```

The output should be this: 
```
path:        rosbag_record.bag
version:     2.0
duration:    15.0s
start:       Nov 14 2021 17:47:16.61 (1636930036.61)
end:         Nov 14 2021 17:47:31.57 (1636930051.57)
size:        184.1 KB
messages:    876
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      147 msgs    : std_msgs/String   
             /rosout       293 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   289 msgs    : rosgraph_msgs/Log 
             /tf           147 msgs    : tf2_msgs/TFMessage
```