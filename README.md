# YuMi controller 
ROS implementation for YuMi control

This package comes as is and use on your risk. 

## Maintainer 
* Gabriel Arslan Waltersson

## Table of Contents
* [General](#general)
* [Dependencies](#dependencies)
* [Usage](#usage)

## General
For more in-depth information and comprehensive installation guide, look in the 
wiki page https://github.com/CRIS-Chalmers/yumi/wiki. This package contains interface and control algorithms for 
the ABB YuMi ROS driver. The core functionality includes inverse kinematics for YuMi in both individual manipulation 
and coordinated manipulation of the arms. There are several ways this package can be used. The idea is that you create a
control class that inherits the yumiControl class and that you can then build functionally on top of it, see 
controller/src/examples/customControllerTutorial.py. There is also a trajectory following controller implemented using
the same structure, see controller/src/examples/trajectoryControl.py. This controller is mainly made to operate with 
slow motions as no dynamical effects have been included in the control loop. The inverse kinematics problem is solved
with heretical quadratic programing (HQP), the HQP implemented using the quadprog solver. The control objectives are
solved together with feasibility objectives (wiki page for more info). The robot is visualized in rviz in real time and
a very simple kinematics simulator is included. 


## Dependencies
For a more comprehensive installation guide take a look at the wiki, where a step-by-step guide from a fresh ubuntu 18 
installation exists. This package uses Ros melodic and python3, ROS and catkin has to be reconfigured for python3
* for python3 ros packages 
```
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy python3-catkin-tools
```

* the geometry2 package needs to be compiled for python3, https://github.com/ros/geometry2

* build orocos_kinematics_dynamics from source in catkin/src
```
https://github.com/orocos/orocos_kinematics_dynamics
```

* python packages, can be installed with pip3
``` 
    numpy
    scipy
    quadprog
```

* for the abb_robot_driver, follow
```
https://github.com/ros-industrial/abb_robot_driver
```

* Command to compile for \catkin_ws folder
``` 
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
``` 

## Usage
The controllers can be launched with roslaunch. 

### Using simulation:

For testing in a simulation, first start the simulator, this can be replaced by a more realistic simulator as long as it
has the same ROS interface as the ABB ros driver. 
``` 
roscore
rosrun controller yumi_simulator.py 
```
### Using hardware:
warning: this activates EGM and joint controllers and also closes EGM and rapid when set_yumi_settings_and_start.py 
is closed (In our lab, Ip: 192.168.125.1) (read wiki before attempting this)
```
roslaunch abb_robot_bringup_examples ex3_rws_and_egm_yumi_robot.launch robot_ip:=<robot controller's IP address> 
rosrun controller set_yumi_settings_and_start.py
```

### Using the trajectory controller:
Then use roslaunch to start the trajectory controller. 
``` 
roslaunch controller yumiTrajectoryControl.launch 
``` 
Then to send example trajectories to the trajectory controller run.
``` 
rosrun controller testTrajectories.py 
``` 
### Running the customControllerTutorial.py:
Only run this file in simulation as it only serves as a demonstration purposes i.e. build your own custom controller for 
your use case. Start by launching the simulation as above. The launch the base.launch, this launches the visualization 
and some background nodes. 
``` 
roslaunch controller base.launch 
``` 
Then to start the controller run.
``` 
rosrun controller customControllerTutorial.py
``` 

