# Jupyter interface for a ROS-based MobileRobot Navigation 
This Jupyter Notebook provides an interface for controlling a mobile robot using the Robot Operating System (ROS). It supports both autonomous and manual driving modes, obstacle avoidance, and also provides controls for resetting the simulation and exiting the program.
The notebook is divided into the following sections:

**Importing Libraries and Initializing Variables:** Necessary Python and ROS libraries are imported, and global variables and ROS publishers are initialized.

**Defining User Interface Elements:** The user interface is created using ipywidgets, which include buttons for driving the robot manually, setting autonomous goals, enabling/disabling obstacle avoidance, resetting the simulation, and exiting the program. Additionally, there are sliders for controlling linear and angular velocities.

**Button Functions:** Functions are defined for each button in the interface. The actions range from controlling robot movements manually to defining an autonomous goal for the robot.

**ROS Callback Functions:** These functions handle incoming data from ROS topics. One is for checking the robot's status related to goal reaching, and another for handling data from a laser scanner for obstacle avoidance.

**User Interface Display:** Here, all the buttons and user interface elements are displayed. The buttons are linked to their respective function, which is triggered when the button is clicked.

**ROS Initialization and Main Function:** The main function of the program, this section initializes the ROS node, defines ROS publishers and subscribers, and displays the initial state of the user interface.

**Trajectory tracking plot:** This function will plot the trajectory tracking of the path that the robot passed.

## Prerequisites

1- Python 3

2- ROS (Noetic)

3- Jupyter Notebook

4- ROS packages: std_srvs, sensor_msgs, move_base_msgs, actionlib_msgs, geometry_msgs

## Usage

The notebook interface is intuitive to use:

**Click the 'Autonomous Drive(Movebase)' button** to set a goal for the robot to reach autonomously. Enter the X and Y coordinates and press 'Send the goal'.


**Click the 'Manually drive(keyboard)' button** to control the robot manually. Use the provided buttons to move the robot and control its speed.


**Use the 'Enable/Disable Obstacles Avoidance'** toggle button to turn on/off obstacle avoidance.
Use the 'Reset' button to reset the simulation.


**Use the 'Exit' button to exit the interface.**


Note: The interface communicates with the ROS system in the backend. Please make sure the appropriate ROS system and simulation are running before using the interface.

## Installation



**Jupyter Notebook**

The Jupyter Notebook is not included with Python,so if you want to try it out, you will need to install
Jupyter.

`pip3 install jupyter bqplot pyyaml ipywidgets
jupyter nbextension enable --py widgetsnbextension`


**JupyterLab**

JupyterLab is the next-generation user interface, including notebooks. It has a modular structure, where you can
open several notebooks or files 

To install it: `pip3 install jupyterlab`

To start it: `jupyter lab --allow-root --ip 0.0.0.0`


**Jupyter Extensions**

• To install some of the most common extensions we can:

- For Jupyter Notebook, run:

`pip3 install jupyter_contrib_nbextensions`

`pip3 install jupyter_nbextensions_configurator`

`jupyter contrib nbextension install`




## Result


https://github.com/dssdanial/SLAM_gmapping_movebase/assets/32397445/4bce4504-4bdf-4377-bb98-f35ec32e374c



