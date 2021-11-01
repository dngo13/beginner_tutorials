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

## Building Instructions
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

## Running the Program
Now that the package is built, we can run the program. 
In this terminal, run ```roscore```.
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