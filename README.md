# RT1 final project: SLAM_gmapping_movebase
================================

A software architecture for the control of the robot in the environment. The software relys on the move_base and gmapping packages for localizing the robot and plan the motion.

In this project, the software architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user’s input):

      1-  autonomously reach a x,y coordinate inserted by the user
      2-  let the user drive the robot with the keyboard
      3-  let the user drive the robot assisting them to avoid collisions


* The Robot can be obsereved in both Rviz and Gazebo environments as follows:
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

After doing this download the package and then you are ready to run the simulation, a launch file is porvided,
so you can simply start the simulation with:
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
This is the first node that I have developed, it has the purpose of showing an interface in which the user can 
choose the modality, reset the simulation or quit the execution of the program. When an input is given the node run
the correct command to launch a node thanks to the `system()` function. 

Here you can find a list of the commands:

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/interface.png" width="500" height="300">
</p>

(I) Autonomous XY-Position Node
----------------------
This node aims to, once a position has been given, drive the robot in the correct position in the environment.
First of all the node requires from the user the `x` and `y` position of the goal, then a message of type `move_base_msgs/MoveBaseActionGoal` is generated
and published in the `/move_base/goal` topic.
Every goal is tracked by the node with its `id`, that is randomly generated inside the node itself.

after establishing the position to be reached the user can at any time cancel the goal (pressing the `q` key) and quit the execurtion (pressing the `s` key).
If one of the above keys is pressed, a message of type `actionlib_msgs/GoalID` is generated and then published in the `/move_base/cancel` topic to cancel the goal.

To know if the robot has reached the goal a `/move_base/status` message handler is implemented. It checks the meassages published on the previously mentioned topic.
In particular, when the robot stops, the status code becomes `3` if the robot has reached the goal position, otherwise the status code becomes `4` if the robot can not
reach the given position.

## Nodes struscture
<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/mode01.jpg" width="600" height="500">
</p>

## video
The result of the first task is shown as the following video:

https://user-images.githubusercontent.com/32397445/182617068-742112e0-2abb-4133-b68c-c55e71dd4b9f.mp4



(II) Manual Drive- _without_ Obstacle Avoidance assistant
----------------------
This node aims to give the user the possibility of moving the robot in the environment using the keyboard. To opportunetly manage robot movement in the
environment I have decided to implement four variables, two used for the velocity values (one for the linear velocity and one for the angular) and two used for the direction

As requested the user can move the robot using the keyboard, each specific key modify the above mentioned variables to drive the robot in the maze.
Once the velocity and the direction has been modified new velocities are published in the `/cmd_vel` topic. 

Below you can find a list of the command to move the robot and increase/decrease velocities:


<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/keyboard.png" width="500" height="300">
</p>

## Nodes struscture

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/mode02.jpg" width="500" height="300">
</p>

## Video
The result of the first task is shown as the following video:


https://user-images.githubusercontent.com/32397445/182617882-0e5dd0a1-4319-4f45-95e8-0d2c0bbfb350.mp4



(III) Manual Drive- _with_ Obstacle Avoidance assistant
----------------------
This is the last node developed, basically it aims to give the user the possibility to drive the robot in the environment using the keyboard, but in this
case we want also to provide automatic obstacle avoidance. Since the goal is partillay similar to what was done with the `KeyboardDrive` node, part of the code
has been re-used for this node. 
To manage obstacle avoidance we have to check what the robot laser scanner see, to do so the node subscribes to the `/scan` topic. This topic provide a `ranges array`
composed of 720 elements. The ranges array is filled with the distance of the obstacles in the `180°` field of view of the robot. Thanks to two functions it is possible to
divide the array in three parts (robot's left right and front) and check the closest obstacle in each section, then if the robot will be close to an obstacle it will be
properly rotated. 


## Nodes struscture

<p align="center">
<img src="https://github.com/dssdanial/SLAM_gmapping_movebase/blob/main/images/mode03.jpg" width="500" height="300">
</p>

## Video
The result of the first task is shown as the following video:


https://user-images.githubusercontent.com/32397445/182617913-dd8b501b-b90d-4474-8985-fd134b660af3.mp4



Flowcharts
----------------------
Here you can find three flowchart for the three driving modalities, the first one is the one regards the ReachPosition node, the second regards the KeyboardDrive node,
the last one regards the KeyboardDriveObs node. 
<p align="center">
<img src="https://github.com/andreamanera/RTassignment3/blob/noetic/images/flowcharts.jpg" width="700" height="500">
</p>

Thanks to the command `rosrun rqt_graph` here you can see a graph of the nodes and their interactions.
<p align="center">
<img src="https://github.com/andreamanera/RTassignment3/blob/noetic/images/rqt_graph.png" width="700" height="500">
</p>

Coclusion and possible improvments
----------------------

I'm pretty satisied with the final result, even if the tasks for this assignment were challenging, all three driving modalities work correctly.
regarding the improvments I have observed that in the ReachPosition modality the feedback to check if the robot has reached the goal takes a lot of time
to be detected by the `base_scan/status` topic, so maybe it could be a good idea to manage the feedback in another way.
