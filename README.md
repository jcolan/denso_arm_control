# denso_arm_control
Denso arm controller for ROS using the b-CAP protocol. This code has been tested with the 6-axis Denso VS050 and Denso controller RC8. The compilation has been verified using catkin-tools in a ROS Kinetic (Ubuntu 16) and ROS Noetic (Ubuntu 20) environments.


## Prerequisites

Prerequisities included in this repository;
* TRAC-IK
* realtime_scheduler_interface


Prerequisities not included in this repository and you need to install independently.

```
sudo apt install ros-noetic-geometry-msgs liborocos-kdl-dev ros-noetic-kdl-parser ros-noetic-eigen-conversions ros-noetic-dynamic-reconfigure ros-noetic-ros-control ros-noetic-joint-limits-interface ros-noetic-actionlib ros-noetic-controller-manager qt5-default 
```

## ROS nodes:
This package includes the following nodes:
* denso_arm_control: contains the low-level controller that handles the b-CAP protocol communication with RC8. It uses a ros-control implementation. It contains the following topics:
    * **/position_controller/command** : (Subscriber) Receives the joint position command
    * **/joint_states** : (Publisher) Publishes the current robot joint positions

* denso_arm_gui: contains the graphical user interface to start the robot control. 

* denso_arm_planner: contains a high-level planner. It requires the [TRAC-IK library](https://bitbucket.org/traclabs/trac_ik/src/master/) and the [Reflexxes TypeII motion planner](https://github.com/Reflexxes) (both are contained in this repository). It requires a URDF model loaded in the ROS parameter server to solve the IK. It provides two ROS Action servers:
    * **/FollowJoint** : Action that receives a sensor_msgs/JointState goal and generates a smooth joint trajectory to the desired goal by using the [Reflexxes library](https://github.com/Reflexxes/RMLTypeII)
    * **/FollowTool** : Action that receives a geometry_msgs/Pose goal, solve the IK by using [TRAC-IK](https://bitbucket.org/traclabs/trac_ik/src/master/) and generates a smooth joint trajectory to the desired goal by using the [Reflexxes library](https://github.com/Reflexxes/RMLTypeII)

* denso_arm_example: contains a ROS Action client to generate sample commands for the **/FollowJoint** action server and the **/FollowTool** action server. Use it as a base to create your own robot controller.

![alt text](https://github.com/jcolan/denso_arm_control/doc/figures/rqt_graph.png "ROS nodes")

## Launch file
The launch file located in **/launch/denso_arm_contro.launch** allows to enable and dsiable the execution of the ROS nodes. It also allows configuration of the follwoing parameters:
* **prefix** : It appends a prefix to the topics and server names. Useful if you are using more than one robot.
* **name** : String robot identifier. Useful if using more than one robot.
* **robot_identifier**: Numerical robot identifier. Useful if using more than one robot.
* **robot_ip**: IP address of the robot. Make sure the local compute and the robot are in the same network (*ifconfig*: to check yout current IP address; *ping robot_ip_address*: to verify your PC is in the same network as the robot) 
* **robot_sim**: Defines if you are using the robot simulator or the real robot. By default is set to True, if you will use the real robot change it to False. *Recommendation*: Try the controller with the simulation first.
* **thread_sampling_time_nsec**: control loop time expressed in nanoseconds.
* **robot_description**: URDF file path. Required for the denso_arm_planner node.
* **robot_config.yaml**: Robot configuration file path with joint and velocity limits. Required for the denso_arm_control node. Modify it according to your robot configuration.
* **pos_control.yaml**: Controller configuration file. Require for the denso_arm_control node. Defined for a 6-DOF robot.
* **controller_spawner**: Initializes the controller manager required for the ros-control package.

## URDF file
A URDF file for the Denso VS050 robot is located in **/urdf/vs050.urdf**. The URDF file is needed for the TRAC-IK and the simualtion environment setup. To generate your own URDF file you can follow [this guide](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file). Additionally, if you have the CAD model of the robot, the [Solidworks URDF Exporter add-in](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file) will be quite useful.

## GUI
This package includes a GUI developed with QT (soure code located in **/src/qt/**).

![alt text](https://github.com/jcolan/denso_arm_control/doc/figures/gui.png "GUI")

* Sequence for activation is:
    1. Connect: Establish a connection with the robot. 
    2. Motor On : Activates the robot motors (Make sure you are in a safe environment). 
    3. Slave Mode: Start the Slave mode for joint position command.
* Sequence for desactivation follows the inverse order:
    1. Slave Mode: It will release the slave mode.
    2. Motor Off: It will stop the motors
    3. Disconnect: It will disconnect the robot

## Dynamic reconfigure
The node denso_arm_planner allows you to modify the maximum join velocity and maximum joint acceleration online through the dynamic reconfigure package. The absolute maximum velocity is set to 200 deg/s and the absolut maximum acceleration is set to 50deg/sec2. By default the maximum velocity and acceleration is set to 50% of the absolute maximum. You can modify this parameter by executing (after launching the denso_arm_planner node):
```
rosrun rqt_reconfigure rqt_reconfigure
```
![alt text](https://github.com/jcolan/denso_arm_control/doc/figures/simulation.png "CoppeliaSim simulation")

The slider varies from 1% to 100%.

## Simulation

The package includes a simulation environment of the VS050 implemented with [CoppeliaSim](https://www.coppeliarobotics.com/) located in:
```
 ./sim/vs050.ttt
```

![alt text](https://github.com/jcolan/denso_arm_control/doc/figures/rqt_reconfigure.png "Dynamic reconfigure")

The simulation model contains the the following ROS topics for communication with denso_arm_control:

* Subscriber:
    * **/sim/joint/cmd**: Receives a joint position command to control the robot

* Publishers:
    * **/sim/joint/state**: Publishes the current joint states
    * **/sim/tool**: Publishes the current tool frame pose

* CoppeliaSim: The simulation environment has created using CoppeliaSim v4.1. It uses the ROS plugin that requires ROS to be launched before initializing CoppleiaSim (i.e. run *roscore* before launching CoppeliaSim)


## Publications
This code has been used as the core controller for the following publications. For further references please follow the links:

* Colan, Jacinto, et al. "[A Cooperative Human-Robot Interface for Constrained Manipulation in Robot-Assisted Endonasal Surgery](https://www.mdpi.com/2076-3417/10/14/4809)." Applied Sciences 10.14 (2020): 4809.

* Colan, Jacinto, et al. "[Optimization-Based Constrained Trajectory Generation for Robot-Assisted Stitching in Endonasal Surgery ](https://www.mdpi.com/2218-6581/10/1/27)." Robotics 10.1 (2021): 27.
