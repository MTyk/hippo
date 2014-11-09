
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>

#define pi 3.14159265

float x=0;
float y=90;
float z=90;

void setJoints(const geometry_msgs::Vector3 jointstate)
{
  ROS_INFO("jointstate y,z: [%f,%f]", jointstate.y,jointstate.z);
  x=jointstate.x;
  y=jointstate.y;
  z=jointstate.z;
}

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "state_publisher" );
  ros::NodeHandle n;

  ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);
  ros::Subscriber sub = n.subscribe("ptu_servo_angles", 1000, setJoints);
  
  ros::Rate loop_rate(15);
  sensor_msgs::JointState js;

	//js.name.push_back(std::string("servo_joint"));
	//js.position.push_back(0);
	
  js.name.resize(3);
  js.position.resize(3);
  js.name[0] ="servo_joint";
  js.position[0] = x;
  js.name[1] ="y_joint";
  js.position[1] = y;
  js.name[2] ="z_joint";
  js.position[2] = z;


	while( n.ok() )
	{
 //   js.header.frame_id="/base_link";
    js.header.stamp = ros::Time::now();
		joint_state_publisher.publish(js);
		loop_rate.sleep();
	}
}
