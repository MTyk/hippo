#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h" 

# define M_PI 3.14159265358979323846

bool danger=false;
bool turnRight=false;
bool turnLeft=false;



void rangeGuard(const sensor_msgs::LaserScan msg){

	//float ranges[]=msg.ranges;
	//ranges has 1080 values (270 degree FOV divided by 0.25 increment step). FOV of 90 degrees is essential (45*0.25+540 to 540-45*0.25)

	for (int i=230;i<310;i++){
		if(msg.ranges[i]<0.600){
			ROS_INFO("Distance: %f",msg.ranges[540]);
			//ROS_INFO("WARNING! Obstacle is near");
			danger=true;
			if(i>270){
				turnRight=true;
				turnLeft=false;
			}else{
				turnLeft=true;
				turnRight=false;
			}
		}	
	}

}

int main(int argc, char **argv){

	ros::init(argc,argv,"talker");
	ros::NodeHandle n;
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1000);
	ros::Subscriber sub = n.subscribe("scan", 1000, rangeGuard);
	
	ros::Rate loop_rate(10);
	int count = 0;
	while (ros::ok()){
		ROS_INFO("Danger: %i",danger);
		geometry_msgs::Twist msg;
		if(danger==false){
			msg.linear.x = 0.25;
			ROS_INFO("%f",msg.linear.x);
			cmd_pub.publish(msg);
		}else{
			for(int i=0;i<90;i++){
				if(turnLeft){
					ROS_INFO("Turning left");
					msg.angular.z=i*M_PI/180;
				}
				if(turnRight){
					ROS_INFO("Turning Right");
					msg.angular.z=-i*M_PI/180;
				}
				cmd_pub.publish(msg);
			}
				danger=false;
		}
		
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}


	return 0;
}
