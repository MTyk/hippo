#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

ros::Publisher cmd_pub;

void relaycmd (const geometry_msgs::Twist cmd){
  geometry_msgs::Twist newcmd;
  
  newcmd.linear.x=cmd.linear.x;
  newcmd.linear.y=cmd.linear.y;
  newcmd.linear.z=cmd.linear.z;
  newcmd.angular.x=cmd.angular.x;
  newcmd.angular.y=cmd.angular.y;
  newcmd.angular.z=cmd.angular.z;
  
  cmd_pub.publish(newcmd);
}


int main(int argc, char **argv){

	ros::init(argc,argv,"cmd_vel_relay");
	ros::NodeHandle n;
  cmd_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1000);
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, relaycmd);
	
	

	 	ros::spin();
 
}
