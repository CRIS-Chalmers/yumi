# yumi (Work in progress!)
ROS implementation for YuMi control


## Maintainer 
* Gabriel Arslan Waltersson

## Table of Contents
* [General](#general)
* [ROS Nodes](#ROSNodes)
* [Dependencies](#dependencies)
* [Usage](#usage)

## General
For more in-depth infromation and comprehensive installation guide, look in the wiki page https://github.com/CRIS-Chalmers/yumi/wiki. This package contains control algorithms for inverse kinematics control of YuMi. The robot can be controlled with two modes. Either with individual manipulation or coordinated maniplation. With individual manipulation the robotic arms are controlled seperatly and with coordinated manipulation the robotic arms are controlled as a single system. This controller is only made to opperate with very slow motions as no dymanical effects have been included in the control loop. The inverse kinematics problem is solved with heieracal quadratic programing (HQP), the HQP implemented is a sequential HQP using the quadprog solver. The control objctives are solved together with feasibility objectives (wiki page for more info). The controller takes trajectory paramters as input and inperpolates between the points using cubic functions. The controller always follow the latest trajectory paramters recived and ingors the current or previous sent. The YuMi bask frame is the base frame for the trajectory paramters. The robot is visulized in rviz in real time and a very simple kinematics simulator is included. 

## ROSNodes
The controller operates over several nodes: kdl_kinematics, tf_broadcaster, controllerMaster and initalPoseJointController (exluding the YuMi drivers and ros robot_state_publisher).

* kdl_kinematics: this node is responible for converting the recived joint positions to forward kinematics and calculate the jacobians. This node uses the kdl kinematics library.   

* contollerMaster: this is the main node that does most of the calculations. IT is responisble for taking trajectory paramters and the infromation from the kinematics node and calcualte the real-time velocity commands for the robot.  
 
* tf_broadcaster: this node add frames to the ROS tf tree. 



## Dependencies
For a more comprehensive installationg guide take a look at the wiki, where a step by step guide from a fresh ubuntu 18 installation exists. This package uses Ros melodic and python3, ROS and catkin has to be reconfigured for python3
* for python3 ros packages 
```
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```

* for tf package to be compiled for python3 one modification might be needed, in geomery2/test_tf2/CMakeList: comment out the line "if(NOT CATKIN_ENABLE_TESTING)" if problems occur during compiling 
```
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
```

* build orocos_kinematics_dynamics from source in catkin/src
```
https://github.com/orocos/orocos_kinematics_dynamics
```

* python pakages, can be installed with pip3
``` 
    numpy
    scipy
    quadprog
```

* for the abb_robot_driver, follow
```
https://github.com/ros-industrial/abb_robot_driver
```

* Comand to complie for \catkin folder,  as some packages are not standard catkin and compile with only cmake
``` 
catkin_make_isolated -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
``` 

## Usage
There are two ways to start the controller. Either the controller is started through the interface script or each node is started manually. If somthing unexpected happens and the ropbot needs to be stopped, always use the physical emegency button on the ABB flexpendant insead of trying to stop it through the command line. Another important warning, there is no guarante that the HQP and inverse kinematics will find a good solution, so always be prepared and do not run the robot at a greater speed then what you are capable to intervene.  

Method 1:

To start the controller through the interface script. 

* open a terminal and start the roscore
``` 
roscore
``` 

* open a second terminal
``` 
rosrun robot_setup_tf Interface.py
``` 
This will launch a very simple comand line interface, were different commands can be entered to start differnet nodes. The interface also gives information about which nodes that are running and what the valid commands are. Importent the arms of the robot are not in the correct place until the simulation or the connection/EGM to the robot is started. For running in simulation type "startSimualtion" and press enter. For running on the real robot assuming that the ethernet is plugged in and the robot is in the correct state (read wiki before attempting this), first type "connectYuMi", this will start the connection but not the EGM conrollers. To start the internal controllers and EGM connection, type "startEGM", once this is active, the robot will follow any velocity commands. To reset the pose type "resetPose", before running this be sure that the robot is not close to anything (such as workspace or objects) as this mode does not handle collision avoidance. Once the robot is in a good position, i.e. after reseting the pose then the main controller can be started with "startController", at this point the robot will follow any trajecory parameters sent to the controller. ShutDown the system, first wait for the robot to not be moving, then type "shutDown". There are other commands to shutDown individual nodes if that is desierd.     

Method 2:

The second method of starting the contorller is to lanuch all the nodes manually in seperate terminals. This may give the user better control over what is running or to make modifications. It is adviced to read through the wiki before attempting this. 

* start yumi_description, this starts the rviz visualization and the robot_state_publisher for visulizing the robot pose, will automatically start a roscore if there isnt one already running.   
``` 
roslaunch yumi_description display.launch
``` 

* start tf broadcaster, for seting up transformation tree between the robot and world frame and the wrist to the tip of the grippers. Can be modified to inlcude transformations to camera and other objects.
``` 
rosrun robot_setup_tf tf_broadcaster
``` 

* start kdl_kinematics, this node calculates the forward and the jacobians from the joint positions. 
``` 
rosrun controller kdl_kinematics
``` 

* For starting a simple simulator, it only acts as a integrator for the joint velocities, but no the less is very useful for testing. 
``` 
rosrun simulation_rviz yumi_simulator.py
``` 

* For robot or robotstudio (warning: this activates egm and joint controllers and also closes egm and rapid when set_yumi_settings_and_start.py is closed) (In our lab, Ip: 192.168.125.1)
```
roslaunch abb_robot_bringup_examples ex3_rws_and_egm_yumi_robot.launch robot_ip:=<robot controller's IP address> 
rosrun robot_setup_tf set_yumi_settings_and_start.py
```

* To reset the pose, make sure the robot is in a good configuration before running this one, will automaticallt teminate when it is done. Do not run this at the same time as the controllerMaster. This is a open loop controller, so accuary is only so high. But the main purpose is to make sure that no joints is saturated, which it is sufficent for. 
``` 
rosrun controller initalPoseJointController.py
``` 

* Start the controller, it is adviced to run the rest pose contorller before starting this one as it is easy to have the wrist joints saturated otherwise, which could give bad results. When this is started the robot will follow the trajectory paramters sent to it.
``` 
rosrun controller controllerMaster.py
``` 

* For testing a script with example trajectories are provided, the trajecory paramters are hard coded in this example but much more advanced planners can be made. 
``` 
rosrun example_trajectories test_trajectories.py
``` 
