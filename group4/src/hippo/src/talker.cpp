#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

# define M_PI 3.14159265358979323846

//constants definition
#define MAX_RANGE 1
#define MIN_RANGE 0.7
#define MAX_SPEED 1
#define MIN_SPEED 0.0

bool danger=false;
bool turnRight=false;
bool turnLeft=false;

float range=MIN_RANGE;
float speed;

void rangeGuard(const sensor_msgs::LaserScan msg)
{

    //float ranges[]=msg.ranges;
    //ranges has 1080 values (270 degree FOV divided by 0.25 increment step). FOV of 90 degrees is essential (45*0.25+540 to 540-45*0.25)
    int rangeIndex;

    range=MAX_RANGE;

    for (int i=230; i<310; i++)
    {
        if(msg.ranges[i]<range)
        {
            range = msg.ranges[i];
	   ROS_INFO("Range %f",range);
            rangeIndex = i;
        }
    }
    if(range<MIN_RANGE)
    {
        //ROS_INFO("WARNING! Obstacle is near");
        danger=true;
        if(rangeIndex>270)
        {
            turnRight=true;
            turnLeft=false;
        }
        else
        {
            turnLeft=true;
            turnRight=false;
        }
    }


}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1000);
    ros::Subscriber sub = n.subscribe("scan", 1000, rangeGuard);

    ros::Rate loop_rate(10);
    int count = 0;

    //variables for the speed controller
    speed = 0;
    float error = 0.25;
    float k_p = 2;    //proportional control parameter
    //geometry_msgs::Twist msg;
    while (ros::ok())
    {
        ROS_INFO("Danger: %i",danger);
        geometry_msgs::Twist msg;

        if(danger==false)
        {
          msg.angular.z=0.0;
          cmd_pub.publish(msg);
            //msg.linear.x = 0.25;
	    ROS_INFO("MIN_RANGE: %f",MIN_RANGE);
	    	ROS_INFO("Error: %f",error);
        ROS_INFO("RANGE: %f",range);
            error = range - MIN_RANGE;
            speed = error * k_p+MIN_SPEED ;
	    msg.linear.x = speed;
            ROS_INFO("Speed %f",msg.linear.x);
            cmd_pub.publish(msg);
        }
        else
        {
//   for(int i=0;i<90;i++){
                if(turnLeft)
                {
                   // ROS_INFO("Turning left");
                    msg.angular.z=0.5;
            		    msg.linear.x=speed;
                }
                if(turnRight)
                {
                   // ROS_INFO("Turning Right");
                    msg.angular.z=-0.5;
   		              msg.linear.x=speed;
                }
                cmd_pub.publish(msg);
//  }
            danger=false;
        }


        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    //msg.linear.x=0.0;
    //cmd_pub.publish(msg);

    return 0;
}
