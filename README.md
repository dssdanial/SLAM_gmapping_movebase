# RT1 final project: SLAM_gmapping_movebase
================================

A software architecture for the control of the robot in the environment. The software relys on the move_base and gmapping packages for localizing the robot and plan the motion.

In this project, the software architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user’s input):

      1-  autonomously reach a x,y coordinate inserted by the user
      2-  let the user drive the robot with the keyboard
      3-  let the user drive the robot assisting them to avoid collisions


* The Robot can be obsereved in both Rviz and Gazebo environments as follows.
<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/map_01.png" width="500" height="300">
</p>

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/map_02_rviz.png" width="500" height="300">
</p>

This represents the point of view from Rviz. Rviz is a 3D visualization tool for ROS applications. It offers a view of the robot model, acquires sensor information from the robot sensors, and reproduces the acquired data. It can display data from video cameras, lasers, 3D and 2D devices, including images and point clouds. To obtain this result the robot must have explored all the surroundings since with the gmapping algorithm we do not have a totale knowledge of the environment, whereas with a pre-existing map we do.


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
$ sudo apt-get install ros-hydro-teleop-twist-keyboard
```

After installing the required packages, it is time to run the simulation. Concequently, a launch file is porvided here as follows.
``` bash
$ roslaunch RTassignment3 main.launch
```



Introduction
----------------------

* Regarding the first goal, the `move_base` pakage requires goal to be sent to the topic `move_base/goal`, by sending a message of type `move_base_msgs/MoveBaseActionGoal`.
* Regarding the goals 2) and 3), it should rely on the `teleop_twist_keyboard`, however, in case 3), the cmd_vel may need to be corrected when the user is going to crash into obstacles. Carefully consider the architecture of the system.
* Concerning the goal 3), the robot should not go forward if there is an obstacle in the front, also should not turn left/right if there are obstacles on the left/right.


Interface Node
----------------------
The purpose of this node is showing an interface in which the user can choose different modes, reset the simulation or quit the execution of the program. When an input is given, the node runs the correct command to launch the related node. 

The interface is simply designed as a list of commands as follows:

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/interface.png" width="500" height="300">
</p>


(I) Autonomous XY-Position Node
----------------------
Once a x-y position receieved by user, this node drives the robot to the desired position in the environment. First of all the node requires from the user the `x` and `y` position of the goal, then a message of type `move_base_msgs/MoveBaseActionGoal` is generated and published in the `/move_base/goal` topic.
Every goal is tracked by the node with its `id`, that is randomly generated inside the node itself.


## Nodes struscture
<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/mode01.jpg" width="600" height="500">
</p>

After establishing the position, the user can at any time cancel the goal (pressing the `q` key) and quit the execurtion (pressing the `s` key).
If one of the above keys is pressed, a message of type `actionlib_msgs/GoalID` is generated and then published in the `/move_base/cancel` topic to cancel the goal.


## video
The result of the **_first_** task is shown as the following video:


https://user-images.githubusercontent.com/32397445/182872817-f4ccb08b-4d68-4531-80b7-29de31634bc2.mp4



In order to check wheather the robot has reached the goal or not, a `/move_base/status` message handler is implemented. It checks the meassages published on the previously mentioned topic. In particular, when the robot stops, the status code becomes `3` if the robot has reached the goal position, otherwise the status code becomes `4` if the robot can not
reach the given position.


(II) Manual Drive- _without_ Obstacle Avoidance assistant
----------------------
This node aims to give the user the possibility of moving the robot in the environment using the keyboard. In order to manage the robot's movement in the
environment four parameters are considred, two used for the velocity values (both linear and angular velocity) and two used for the directions.
Each specific key modifies the above mentioned variables to drive the robot through the maze. Once the velocity and the direction has been modified new velocities are published in the `/cmd_vel` topic. 

An interface has been considered as a list of the commands to move the robot and increase/decrease velocities as follows:


<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/keyboard.png" width="500" height="300">
</p>

## Nodes struscture

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/mode02.jpg" width="500" height="300">
</p>

## Video
The result of the **_second_** task is shown as the following video:


https://user-images.githubusercontent.com/32397445/182872919-e427823f-907b-438d-9582-4881dcccf7d8.mp4




(III) Manual Drive- _with_ Obstacle Avoidance assistant
----------------------
This is the last node developed, basically it aims to give the user the possibility to drive the robot in the environment using the keyboard, but in this
case it is also necessary to provide obstacle avoidance autonomously. Since the goal is partially similar to what was done with the previous node, part of the code
has been re-used for this node. 
To manage obstacle avoidance the robot laser scanner is used,and the node subscribes from the `/scan` topic. This topic provide a `ranges array` composed of 720 elements. The ranges array is filled with the distance of the obstacles in the `180°` field of view of the robot. It is possible to divide the array in three parts (robot's left right and front) and check the closest obstacle in each section, then if the robot will be close to an obstacle it will be properly rotated. 


## Nodes struscture

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/mode03.jpg" width="500" height="300">
</p>

## Video
The result of the **_third_** task is shown as the following video:


https://user-images.githubusercontent.com/32397445/182872987-ba6ce54d-a143-4cc3-a866-5fc8f35cb515.mp4





Pseudo code
----------------------
 ## control of the robot
 ``` bash
 listen for UI node's commands

  if autonomous drive
    receive coordinates
    start driving action with coordinates

  elseif manual drive
    redirect teleop_twist_keyboard node to cmd_vel topic

  elseif assisted drive
    run collision avoidance on teleop_twist_keyboard's commands
    send filtered commands to cmd_vel

  send info to Interface node
```


Coclusion and possible improvments
----------------------

All proposed algorithms are executed correctly and the robot completed tasks as well. Results are satisfying, however, some improvements can be achieved.
* reaching a desired point seems taking some delays, so, this could be modified by tuning parameters. 
* A queue could be implemented where goals are reached sequentially.
* When the map of the environment is completed, the robot is not able to understand the priori points whether it is reachable or not, which could be improved in further developments.



