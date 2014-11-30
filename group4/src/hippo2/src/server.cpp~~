#include "ros/ros.h"
#include "std_srvs/Empty.h"


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
    ros::init(argc, argv, "server");
    ros::NodeHandle n;

    ros::ServiceServer pickupService = n.advertiseService("pickup", pickup);
    ros::ServiceServer dropService = n.advertiseService("drop", drop);
    ros::Rate loop_rate(0.2);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
