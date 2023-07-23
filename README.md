# RT2 final project: SLAM_gmapping_movebase
================================

A software architecture for the control of the robot in the environment. The software relies on the move_base and gmapping packages for localizing the robot and planning the motion.

In this project, the software architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user’s input):

      1-  autonomously reach a x,y coordinate inserted by the user
      2-  let the user drive the robot with the keyboard
      3-  let the user drive the robot assisting them to avoid collisions


* The Robot can be observed in both Rviz and Gazebo environments as follows.
<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/map_01.png" width="500" height="300">
</p>

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/map_02_rviz.png" width="500" height="300">
</p>

This represents the point of view of Rviz. Rviz is a 3D visualization tool for ROS applications. It offers a view of the robot model, acquires sensor information from the robot sensors, and reproduces the acquired data. It can display data from video cameras, lasers, 3D and 2D devices, including images and point clouds. To obtain this result the robot must have explored all the surroundings since with the gmapping algorithm we do not have a total knowledge of the environment, whereas with a pre-existing map, we do.


## How to Install and run
----------------------
At first, some packages are required to be installed, if they aren't already installed,it is possible to setup them by the following commands.

* Install xterm:
``` bash
$ sudo apt install xterm
```
* Using the SLAM-Gmapping package:
``` bash
$ git clone https://github.com/CarmineD8/slam_gmapping.git
```
* using the Navigation package:
``` bash
$ sudo apt-get install ros-<your_ros_distro>-navigation
```
* Install teleop-twist-keyboard package:
``` bash
$ sudo apt-get install ros-<your_ros_distro>-teleop-twist-keyboard
```

After installing the required packages, it is time to run the simulation. Consequently, a launch file is provided here as follows.
``` bash
$ roslaunch RTassignment3 main.launch
```



Introduction
----------------------

* Regarding the first goal, the `move_base` package requires the goal to be sent to the topic `move_base/goal`, by sending a message of type `move_base_msgs/MoveBaseActionGoal`.
* Regarding goals 2) and 3), it should rely on the `teleop_twist_keyboard`, however, in case 3), the cmd_vel may need to be corrected when the user is going to crash into obstacles. Carefully consider the architecture of the system.
* Concerning goal 3), the robot should not go forward if there is an obstacle in the front, also should not turn left/right if there are obstacles on the left/right.


## Interface Node

<td><img src="https://github.com/dssdanial/SLAM_gmapping_movebase/assets/32397445/19f9312d-369c-44b1-95f3-fad03c22ef0c" width=270 height=480></td>
<td><img src="https://github.com/dssdanial/SLAM_gmapping_movebase/assets/32397445/c098e8bd-2007-431d-9b95-b22202acbaa9" width=700 height=380></td>


## Documentation
**----------------------**

In order to generate the documentation for this project, Doxygen has been used.
![screen](https://github.com/dssdanial/SLAM_gmapping_movebase/assets/32397445/5fd08cfc-f5ba-4e2e-85d5-7e22c0aedb07)


You can view the Doxygen documentation by clicking [here](https://dssdanial.github.io/SLAM_gmapping_movebase/). 



## Link to Jupyter Notebook file
Here is the link to find the codes:

https://github.com/dssdanial/SLAM_gmapping_movebase/blob/Interface_JupyterROS/scripts/Interface_JupyterROS.ipynb

