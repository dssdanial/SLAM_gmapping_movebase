/**
*
* Subscribes to: <BR>
* /scan to detect obstacles
*
* Publishes to:<BR>
*  /cmd_vel to publish the velocity of the robot
*
* Description: 
*
* This node aims e user the possibility to drive the robot in the environment using
* the keyboard, but in this case we want also to provide automatic obstacle avoidance.
* Since the goal is partillay similar to what was done with the KeyboardDrive node,
* part of the code has been re-used for this node.
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <termios.h>


ros::Publisher publisher;///< Global publisher

//defining variables for veloocity
double velocity = 0.2;///< Global variable for linear velocity
double turn_velocity = 0.5;///< Global variable for turn velocity

// defining variables used to define the direction
int linear = 0; ///< Global variable for linear direction
int angular =0; ///< Global variable for angular direction

// defining threshold for wall distance
double th = 1;///< Global threshold for wall distance

// defining variable for Twist
geometry_msgs::Twist vel;///< Global overall velocity

// function to have non-blocking keyboard input
// (avoid pressing enter)

/**
* \brief function to have non-blocking keyboard input
* \return ch the input given from keyboard
*
* This function is needed to avoid pressing enter after giving an input from keyboard, 
* this function is part of teleop_twist_keyboard.cpp.
**/
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

// function used to find the closest obstacle in each section of the ranges array

/**
* \brief Distance function to check the minimum value in a fixed range of an array
* \param obs_dist[] the array 
* \param val_min the index from which we start the search
* \param val_max the index at which we stop the search
* \return dist_min the minimum value in the range
*
* This function through a for loop compare the elements of the selected range
* and returns the minimum one 
**/
float Distance(float obs_dist[], int val_min, int val_max)
{

	//as the minimum distance is set a large value that will then be compared
	//with all the values between val_min and val_max in the ranges array
	
	float dist_min = 30.0;
	
	//thanks to the for loop and the if statement all values in the ranges
	//array are compared so as to find the smallest
	
	for(int i = val_min; i <= val_max; i++){
	
		if(obs_dist[i] <= dist_min){
		
			dist_min = obs_dist[i];
		}
	}
	
	return dist_min;
}

/**
* \brief check_walls to select the closest obstacle and decide what to do
* \param msg contains the variables provided by the robot laser
*
* This function is used to check where the closest obstacle is (front left or right of the robot).
* Once we know where the closest obstacle is we can decide how to opportunetly modify
* the velocity of the robot.
**/
void check_walls(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//variables for the closest obstacle on robot right, left and front
	float dist_min_right, dist_min_left, dist_min_front;
	
	// array wich will be filled with the elements of ranges
	float scanner[721];
	
	// thanks to this loop scanner array is filled with the element of ranges
	for(int i = 0; i < 722; i++){
		scanner[i] = msg->ranges[i];
	}
	
	// Call at the Distance function to calculate the minimum
	// distance from the obstacles on robot front, right and left
	dist_min_right = Distance(scanner, 0, 159);
	dist_min_left = Distance(scanner, 561, 720);
	dist_min_front = Distance(scanner, 281, 440);
	
	 //warn user
	if ((dist_min_front < th && linear > 0) || ((dist_min_left < th) && angular > 0) || ((dist_min_right < th) && angular < 0))
		std::cout << "Wall detected!\n";
		         

	//if the nearest wall in front of the robot is too close, the robot can only turn
	if (dist_min_front < th && linear > 0){
	    
		linear = 0;
	}
	
	//if the nearest wall on the left or on the right is too close, the robot can only go straight
	if (((dist_min_left < th) && angular > 0) || ((dist_min_right < th) && angular < 0)){
	
		angular = 0;
	}
	
	// compute new velocities
   	vel.linear.x = velocity * linear;
    	vel.angular.z = turn_velocity * angular;
    
    	// publish the new velocity
    	publisher.publish(vel);
}

// function used to associate the input from keyboard with the correct command

/**
* \brief function to associate the input from keyboard with the correct command
* \param input the input that we want to associate with the correct command
*
* This function is used to opportunetly manage the velocity, the input given by the user 
* is associated with a specific operation thanks to a switch and so the velocity and the turn velocity are 
* opportunetly modified
**/
void choose_input(char input)
{

    switch (input)
    {
	    case 'A':
	    case 'a':
		velocity *= 1.1;
		turn_velocity *= 1.1;
            break;
            
            case 'D':
	    case 'd':
		velocity *= 0.9;
		turn_velocity *= 0.9;
            break;
            
            case 'W':
	    case 'w':
		velocity *= 1.1;
	    break;
	    
	    case 'S':
	    case 's':
		velocity *= 0.9;
	    break;
	    
	    case 'Q':
	    case 'q':
		turn_velocity *= 1.1;
	    break;
	    
	    case 'C':
	    case 'c':
		turn_velocity *= 0.9;
	    break;
	    
	    case 'R':
	    case 'r':
		velocity = 0.5;
		turn_velocity = 1;
	    break;
	    
	    case 'Y':
	    case 'y':
		linear = 1;
		angular = 1;
	    break;
	    
	    case 'U':
	    case 'u':
		linear = 1;
		angular = 0;
	    break;
	    
	    case 'I':
	    case 'i':
		linear = 1;
		angular = -1;
	    break;
	    
	    case 'H':
	    case 'h':
		linear = 0;
		angular = 1;
	    break;
	    
	    case 'J':
	    case 'j':
		linear = 0;
		angular = 0;
	    break;
	    
	    case 'K':
	    case 'k':
		linear = 0;
		angular = -1;
	    break;
	    
	    case 'B':
	    case 'b':
		linear = -1;
		angular = 1;
	    break;
	    
	    case 'N':
	    case 'n':
		linear = -1;
		angular = 0;
	    break;
	    
	    case 'M':
	    case 'm':
		linear = -1;
		angular = -1;
	    break;
	    
	    case 'E':
	    case 'e': 

		
		vel.angular.z = 0;
		vel.linear.x = 0;
		publisher.publish(vel);
		system("clear");
		ros::shutdown();
		exit(0);
            break;
            
	    default:
		linear = 0;
		angular = 0;
	    break;
	}
}

// function used to display the menu and compute the velocity with the input taken by the keyboard

/**
* \brief get_input function to display commands and compute the velocity
*
* This function first of all display the menu with the commands with which the user can
* modify the direction and the velocity then call the function choose input to compute the new
* velocity and then asign the computed velocity to the elements of the variable vel
**/
void get_input()
{

    //display instructions
    std::cout << "Drive the robot in the envitonment using your keyboard\n";

    std::cout << R"(Move the robot using these commands as you are using a joystick
    
                y    u    i
                h    j    k
                b    n    m
                
Press:
a : increase angular and linear velocities by 10%
d : decrease angular and linear velocities by 10%
r : reset angular and linear velocities at the default values
w : increase linear velocity by 10%
s : decrease linear velocity by 10%
q : increase angular velocity by 10%
c : decrease angular velocity by 10%
press e to quit

ALL OTHER KEYS WILL STOP THE ROBOT
	)";
    // wait for user input
    char input;
    input = getch();

    // call the function choose_input to decide what to do with the input value received
    choose_input(input);

    // compute new velocities
    vel.linear.x = velocity * linear;
    vel.angular.z = turn_velocity * angular;
    
    // publish the new velocity
    publisher.publish(vel);

    system("clear");
}

/**
* \brief main
*
*
* \param  argc An integer argument count of the command line arguments
* \param  argv An argument vector of the command line arguments
* \return an integer 0 upon success
*
* The main function contains the core part of the program
**/
int main(int argc, char **argv)
{
    system("clear");
    ros::init(argc, argv, "Manual_ObsAv");
    ros::NodeHandle node_handle;
    
    //subribes to /scan topic
    ros::Subscriber subscriber = node_handle.subscribe("/scan", 500, check_walls);

    //this node will publish updated into /cmd_vel topic
    publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // while loop used to get input endless
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (true)
        get_input();
	spinner.stop();
    return 0;
}
