#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "hippo2/Int.h"

bool locateFlag = true;
bool idFlag = true;


bool locateToy(hippo2::Int::Request  &req , hippo2::Int::Response &res)
{
  ROS_INFO("service locate_toy called");
  if(locateFlag){
	res.x = 1;	//toy found
  }else{
  	res.x = 0;
  }	
  locateFlag = !locateFlag;
  return true;
}

bool idToy(hippo2::Int::Request  &req , hippo2::Int::Response &res)
{
  ROS_INFO("service id_toy called");
  if(idFlag){
	res.x = 1;	//type 1
  }else{
  	res.x = 2;
  }	
  idFlag = !idFlag;
  return true;
}

bool pickup(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("service pickup called");
  return true;
}

bool drop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("service drop called");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_server");
  ros::NodeHandle n;

  ros::ServiceServer pickupService = n.advertiseService("pickup", pickup);
  ros::ServiceServer dropService = n.advertiseService("drop", drop);
  ros::ServiceServer locateToyService = n.advertiseService("locate_toy", locateToy);
  ros::ServiceServer idToyService = n.advertiseService("id_toy", idToy);
ros::Rate loop_rate(0.2);
  while(ros::ok()){
	ros::spinOnce();
	loop_rate.sleep();
}

  return 0;
}

