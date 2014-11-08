
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>

#define pi 3.14159265

float x;
float y;
float z;

void setJoints(const geometry_msgs::Vector3 jointstate)
{
  ROS_INFO("r: [%f]", jointstate.x);
 // x=jointstate.x;
 // y=jointstate.y;
 // z=jointstate.z;
}

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "state_publisher" );
  ros::NodeHandle n;

  ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);
  ros::Subscriber sub = n.subscribe("chatter", 1000, setJoints);
  
  ros::Rate loop_rate(15);
  sensor_msgs::JointState js;

	//js.name.push_back(std::string("servo_joint"));
	//js.position.push_back(0);
	
  js.name.resize(3);
  js.position.resize(3);
  js.name[0] ="servo_joint";
  js.position[0] = 0;
  js.name[1] ="y_joint";
  js.position[1] = 90;
  js.name[2] ="z_joint";
  js.position[2] = 90;


	while( n.ok() )
	{
 //   js.header.frame_id="/base_link";
    js.header.stamp = ros::Time::now();
		joint_state_publisher.publish(js);
		loop_rate.sleep();
	}
}
