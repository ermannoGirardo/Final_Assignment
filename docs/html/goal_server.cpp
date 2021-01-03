#include "ros/ros.h"
#include "final_assignment/Random_Goal.h"



/**
* /Function callBack executed when client calls
* 
* /Parameters:
*	an instance for the request(not used) and an instance for the reply of the srv Random_Goal.srv
* 
* /Comments about the code:
*
*				Generate a random number and indices the array of coordinates
*
*				Set the response of the service with the coordinates
*
*				ROS_INFO --> print on screen string between ""
*/
bool target_pos(final_assignment::Random_Goal::Request &req, final_assignment::Random_Goal::Response &res){
	float x_coordinates[]={-4,-4,-4, 5, 5,5};
	float y_coordinates[]={-3, 2, 7,-7,-3,1};
	int random=rand()%6;
	res.x=x_coordinates[random];
	res.y=y_coordinates[random];	
        ROS_INFO("The Coordinates of the new target are: [%f] [%f]", res.x,res.y);
	return true;
}




/**
* /Main function:
*
* /Parameters:
*	
*	int argc, char **argv are mandatory for cpp files
* 
* /Comments about the code:
*
*	ros::init-->initialisation of the node position_server
*
*	ros::NodeHandle n --> set-up nodeHandle
*
*	ros::ServiceServer service= n.advertiseService("robot/random_target",target_pos); --> define server and specify function callBack
*
*	ros::spin()--> blocks the main thread from exiting until ROS invokes a shutdown (for example Ctrl+C)
*/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "goal_server");	
   ros::NodeHandle n;	
   ros::ServiceServer service= n.advertiseService("robot/random_target",target_pos);  
   ros::spin();
   return 0;
}
