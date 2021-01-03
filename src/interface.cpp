#include "ros/ros.h"
#include "stdlib.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "final_assignment/Random_Goal.h"
#include "std_srvs/SetBool.h"

/**
*Define Publisher that will publish velocity of robot on topic move_base/goal
*/
ros::Publisher pub;

/**
*Define the subscriber that will publish on topic /odom
*/
ros::Subscriber sub;

/**
*Instance target of topic move_base_msgs::MoveBaseActionGoal
*
*This will be able to set the coordinates of the goal and Robot will reach it
*/
move_base_msgs::MoveBaseActionGoal target;

/**
*Define global two float variables for the instantaneous position of the robot
*/
float x_pos;
float y_pos;

/**
*random_target a global instance of service Random_Goal
*/
final_assignment::Random_Goal random_target;

/**
*wall_follow a global instance of std_srvs that will call wall_follow service
*/
std_srvs::SetBool wall_follow;



/**
* /Function positionCallBack called when subscriber receives the position
*
* /-Parameter:
* 	
*	msg: declared as constant pointer of topic Odom (to establish the position of the robot)
*	
* /The funcion save on global variables the position of the Robot so i can reuse it on main
*
*/

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
	x_pos= msg->pose.pose.position.x;
	y_pos= msg->pose.pose.position.y;
}	




/**
*MAIN FUNCTION
*
* /Parameters:
*	int argc, char **argv are mandatory for cpp files
*
* /Comments about the code:
*
*	Declare a Rate of 1 Hz for  call periodically ros::spinOnce::
*
* 	pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal",1000) --> Define a Publisher on topic "move_base/goal"
*
*	sub=n.subscribe("/odom",1000,positionCallback) --> Define a Subscriber that publish on topic /odom the position
*
*	x_coordintes and y_coordinates are the two array of possibles coordinates
*
*	Declare client1 of type final_assignment::Random_Goal that will do the request on robot/random_target service
*	
*	Declare client2 of type std_srvs::SetBool that will do the request on wall_follower_switch service
*
*	if(command==1)
*
*		with client1 call pull a random_target and publish on move_base
*
*	if(command==2)
*
*		user choose one of possible coordinates and publish it on move_base
*
*	if(command==3)
*
*		set the argument of the request=true and with client2 call the wall_follow service
*
*	if(command==4)
*
*		set the argument of the request=false and publish on move_base the current position to stop the Robot
*/ 
 
int main(int argc, char **argv)
{
  	ros::init(argc,argv,"interface");		
   	ros::NodeHandle n;		
	ros::Rate r(1);
   	pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal",1000); 
   	sub=n.subscribe("/odom",1000,positionCallback);
   	float command;
   	float desired_x;
   	float desired_y;
   	float x_coordinates[]={-4,-4,-4, 5, 5,5};
   	float y_coordinates[]={-3, 2, 7,-7,-3,1};
	bool flag=false;
	bool generate_target;
	ros::ServiceClient client1 = n.serviceClient<final_assignment::Random_Goal>("robot/random_target");
	ros::ServiceClient client2 = n.serviceClient<std_srvs::SetBool>("/wall_follower_switch");
   	while(command!=-1)
 	{
		ROS_INFO("Possibles Coordinates for x: {-4,-4,-4, 5, 5,5}");
		ROS_INFO("Possibles Coordinates for y: {-3, 2, 7,-7,-3,1}");
		ROS_INFO("Choose the desired options:");
		ROS_INFO("1):Move randomly to one of the possibles coordinates");
		ROS_INFO("2):Digit the desired coordinates(but you can choose only the coordinates above)");
		ROS_INFO("3):Start following the external walls");
		ROS_INFO("4):Stop the Robot");
		ROS_INFO("-1):Stop the execution of the program");
   		scanf("%f",&command);
		ROS_INFO("The position of the Robot is: [%f];[%f]",x_pos,y_pos);
		ros::spinOnce();
		r.sleep();	
		if(command==1)
		{
			generate_target=client1.call(random_target);
			target.goal.target_pose.header.frame_id="map";
			target.goal.target_pose.pose.orientation.w=1;
			target.goal.target_pose.pose.position.x=random_target.response.x;
			target.goal.target_pose.pose.position.y=random_target.response.y;
			ROS_INFO("Target Position [%f,%f]",random_target.response.x,random_target.response.y);
			pub.publish(target);
			while((abs(random_target.response.x-x_pos)>=0.1)||(abs(random_target.response.y-y_pos)>=0.1))
			{	
				ROS_INFO("Robot Position [%f,%f]",x_pos,y_pos);
				ros::spinOnce();
				r.sleep();
				
			}
		}
   		if(command==2)
   		{
			
			ROS_INFO("Digit the x coordinate");
			scanf("%f",&desired_x);
			ROS_INFO("Digit the y coordinate");
			scanf("%f",&desired_y);
			//Si verifichi che è una coordinata possibile:
			if((desired_x==x_coordinates[0])&&(desired_y==y_coordinates[0]))
				flag=true;
			if((desired_x==x_coordinates[1])&&(desired_y==y_coordinates[1]))
				flag=true;
			if((desired_x==x_coordinates[2])&&(desired_y==y_coordinates[2]))
				flag=true;
			if((desired_x==x_coordinates[3])&&(desired_y==y_coordinates[3]))
				flag=true;
			if((desired_x==x_coordinates[4])&&(desired_y==y_coordinates[4]))
				flag=true;
			if((desired_x==x_coordinates[5])&&(desired_y==y_coordinates[5]))
				flag=true;
			if(flag==true)
			{
				target.goal.target_pose.header.frame_id="map";
				target.goal.target_pose.pose.orientation.w=1;
				target.goal.target_pose.pose.position.x=desired_x;
				target.goal.target_pose.pose.position.y=desired_y;
				pub.publish(target);
				while((abs(desired_x-x_pos)>=0.1)||(abs(desired_y-y_pos)>=0.1))
				{
					ROS_INFO("The position of the Robot is: [%f];[%f]",x_pos,y_pos);
					ros::spinOnce();
					r.sleep();
				}
			}
			else
			{	
				ROS_INFO("Impossible to reach the coordinate!!");
			}
			flag=false;
		}
		if(command==3)
		{
			ROS_INFO("Robot will start to follow the walls!");
			wall_follow.request.data=true;
			client2.call(wall_follow);
			
		}
		if(command==4)
		{
			wall_follow.request.data=false;
			client2.call(wall_follow);
			target.goal.target_pose.header.frame_id="map";
			target.goal.target_pose.pose.orientation.w=1;
			target.goal.target_pose.pose.position.x=x_pos;
			target.goal.target_pose.pose.position.y=y_pos;
			pub.publish(target);
			ROS_INFO("Robot stopped");
		}
    	}
	ros::spin();
    	return 0;
}     

