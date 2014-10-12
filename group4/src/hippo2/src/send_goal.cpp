#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

int main(int argc, char** argv){

	if (argc<3){
		ROS_ERROR("You must specify the pose of the robot (x,y,theta)!");
	}

	ros::init(argc,argv,"send_goals");
	ros::NodeHandle nh;

	double x=atof(argv[1]);
	double y=atof(argv[2]);
	double theta=atof(argv[3]);

/*	double x=0.0;
	double y=0.0;
	double theta=0.0;
*/
	ROS_INFO("The goal is x=%f y=%f theta=%f",x,y,theta);

	MoveBaseClient ac("move_base",true);

	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

    // Convert the Euler angle to quaternion
	double radians = theta * (M_PI/180);
	
	tf::Quaternion quaternion;
	quaternion = tf::createQuaternionFromYaw(radians);
	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);
	
	goal.target_pose.pose.orientation = qMsg;

	ROS_INFO("Sending goal x = %f, y = %f, theta = %f", x, y, theta);
	ac.sendGoal(goal);
	// Wait for the action to return
	ac.waitForResult();
	
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
 		ROS_INFO("You have reached the goal!");
	else
		ROS_INFO("The base failed for some reason");
	return 0;
}
