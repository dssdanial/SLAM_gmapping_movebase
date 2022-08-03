/**
* Services:<BR>
*  /gazebo/reset_simulation used to reset the simulation in gazebo
*
* Description: 
*
* This is the first node developed, it has the purpose of showing an 
*interface in which the user can choose the modality, reset the simulation 
*or quit the execution of the program. When an input
* is given the node run the correct command to launch
* a node thanks to the system() function.
**/
#include "ros/ros.h"
#include "std_srvs/Empty.h"


std_srvs::Empty reset; ///< Global variable to reset the simulation

std::string menu = R"(
        
______________ Choose the simulation mode____________
_____________________________________________________|
|1  |        Go to desire x-y position               |
|___|________________________________________________|
|2  |         keyboard-manually drive                |
|___|________________________________________________|
|3  |keyboard-manually drive with obstacles avoidance|
|___|________________________________________________|
|4  |                Reset                           |
|___|________________________________________________|
|0  |                Exit                            |
|___|________________________________________________|
)"; ///< Global variable to print the menu


/**
* \brief function used to display the menu, take the input from keyboard
* \return modality the input taken from keyboard
*
* The function print a short message and the menu showing all the possible commands
* and then take the input of the user from keyboard to run a specific modality
**/
int interface()
{
    // The system() function is a part of the C/C++ standard library. It is used to pass the commands that can be executed
    // in the terminal of the operating system, returns the command after it has been completed.
    
    // to clear the console
    system("clear");

    std::cout << "WELCOME!\n";
    std::cout << menu;

    int input; 

    input = getchar();

    return input; 
}

/**
* \brief main
*
*
* \param  argc An integer argument count of the command line arguments
* \param  argv An argument vector of the command line arguments
* \return an integer 0 upon success
*
* The main function through a while loop associate a specific input taken from keyboard to a specific modality which
* is executed thanks to the system function
**/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "interface");

    
    while (true)
    {
	// here the input taken is associated with the correct modality
        switch (interface())
        {
		case '1':
		    //launch x-y position node
		    system("rosrun RTassignment3 xy_position");
		break;
		
		case '2':
		    //launch KeyboardDrive node
		    system("rosrun RTassignment3 KeyboardDrive");
		break;
		
		case '3':
		    //launch KeybooardDriveObs node
		    system("rosrun RTassignment3 KeyboardDriveObs");
		break;
		
		case '4':
		    //call gazebo/reset_simulation service and reset the simulation
		    ros::service::call("/gazebo/reset_simulation", reset);
		break;
		
		case '0':
		    //clear the screen and exit from main
		    system("clear");
		    return 0;
		break;
		
		default:
		
		break;
        }
    }

    return 0;
}
