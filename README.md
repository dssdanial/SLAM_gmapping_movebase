# RT1 final project
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


## How to Instal and run
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


# Nodes description
## Rviz node
When launching Rviz, three nodes are actually executed:

* joint_state_publisher: the package reads the robot_description parameter from the parameter server, finds all of the non-fixed joints and publishes a JointState message with all those joints defined. If GUI is present, the package displays the joint positions in a window as sliders.

* robot_state_publisher: the package uses the URDF specified by the parameter robot_description and the joint positions from the topic joint_states to calculate the forward kinematics of the robot and publish the results via tf.

* rviz


## Slam_gmapping node
This ROS node is contained in the `gmapping` package and provides laser-based SLAM (Simultaneous Localization and Mapping). Using `slam_gmapping`, you can create a 2-D occupancy grid map from laser and pose data collected by a mobile robot. To use `slam_gmapping`, you need a mobile robot that provides odometry data and is equipped with a horizontally-mounted, fixed, laser range-finder. The `slam_gmapping` node will attempt to transform each incoming scan into the `odom` (odometry) `tf` frame. The `slam_gmapping` node takes in `sensor_msgs/LaserScan` messages and builds a map (`nav_msgs/OccupancyGrid`). The map can be retrieved via a ROS topic or service.

## Move_base node
The move_base package provides an implementation of an action that, given a goal in the world, will attempt to reach it with a mobile base. Actions are services which are not executed atomically, and thus may also offer some additional tools, such as a continuous feedback and the possibility of cancel the request. For the purpose of this project the publish/subscribe architecture will be used for using actions, by sending a message on the topic `move_base/goal`.




``` bash
Reading from the keyboard and Publishing to Twist!
  ---------------------------
  Moving around:
     u    i    o
     j    k    l
     m    ,    .
  For Holonomic mode (strafing), hold down the shift key:
  ---------------------------
     U    I    O
     J    K    L
     M    <    >
  t : up (+z)
  b : down (-z)
  anything else : stop
  q/z : increase/decrease max speeds by 10%
  w/x : increase/decrease only linear speed by 10%
  e/c : increase/decrease only angular speed by 10%
  CTRL-C to quit
```


The commands for moving the robot are as follows:

u to turn left

i to go straight ahead

o to turn right

j to turn counterclockwise

k to stop

l to turn clockwise

m to turn left going backwards

, to go straight back

. to turn right going backwards

The other commands described in the message can be used to change the speed of the robot.







Introduction
----------------------
Once you have launched the simulation you will see an interface in which you can choose the modality to run the robot.
To meet all the requests I have decided to implement four nodes, one for the user interface, and three for the different driving modalities.






interface
----------------------
This is the first node that I have developed, it has the purpose of showing an interface in which the user can 
choose the modality, reset the simulation or quit the execution of the program. When an input is given the node run
the correct command to launch a node thanks to the `system()` function. 

Here you can find a list of the commands:

<center>

| Command | Result |
|:--------:|:----------:|
|__1__   |__Run the node to autonomousely reach a given position__|
|__2__   |__Run the node to drive the robot with the keyboard__|
|__3__   |__Run the node to drive the robot with the keyboard with obstacles avoidance__|
|__4__   |__Reset the simulation__|
|__0__   |__Quit the execution of the program__|

</center>

ReachPosition
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

KeyboardDrive
----------------------
 
**NOTE:** for the remaining two nodes I have decided to implement a function to have non-blocking keyboard input, it is part of [teleop_twist_keyboard.cpp](http://docs.ros.org/en/kinetic/api/teleop_twist_keyboard_cpp/html/teleop__twist__keyboard_8cpp_source.html).

This node aims to give the user the possibility of moving the robot in the environment using the keyboard. To opportunetly manage robot movement in the
environment I have decided to implement four variables, two used for the velocity values (one for the linear velocity and one for the angular) and two used for the direction
(again onefor the linear velocity and one for the angular). 

As requested the user can move the robot using the keyboard, each specific key modify the above mentioned variables to drive the robot in the maze.
Once the velocity and the direction has been modified new velocities are published in the `/cmd_vel` topic. 

Below you can find a list of the command to move the robot and increase/decrease velocities:

* Use these commands as you are using a joystick to change the robot direction:

<center>

| __y__  |__u__  |__i__  |
|:--------:|:--------:|:--------:|
|__h__   |__j__ |__k__   |
|__b__   |__n__ |__m__   |


</center>


* Here commands to increase/decrease linear and angular velocity:

| Command | Result |
|:--------:|:----------:|
|__a__   |__to increase both linear and angular velocity by 10%__|
|__d__   |__to decrease both linear and angular velocity by 10%__|
|__r__   |__to reset both linear and angular velocities to their default values__|
|__w__   |__to increase only linear velocity by 10%__|
|__s__   |__to decrease only linear velocity by 10%__|
|__q__   |__to increase only angular velocity by 10 %__|
|__c__   |__to decrease only angular velocity by 10 %__|

</center>

KeyboardDriveObs
----------------------
This is the last node developed, basically it aims to give the user the possibility to drive the robot in the environment using the keyboard, but in this
case we want also to provide automatic obstacle avoidance. Since the goal is partillay similar to what was done with the `KeyboardDrive` node, part of the code
has been re-used for this node. 
To manage obstacle avoidance we have to check what the robot laser scanner see, to do so the node subscribes to the `/scan` topic. This topic provide a `ranges array`
composed of 720 elements. The ranges array is filled with the distance of the obstacles in the `180°` field of view of the robot. Thanks to two functions it is possible to
divide the array in three parts (robot's left right and front) and check the closest obstacle in each section, then if the robot will be close to an obstacle it will be
properly rotated. 

**NOTE:** this last mentioned part of the code is similar to what done in the [second assignment](https://github.com/andreamanera/RTassignment2)

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
