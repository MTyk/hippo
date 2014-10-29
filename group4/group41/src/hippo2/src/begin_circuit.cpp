#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <stdlib.h>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

/**
Moves the robot to given coordinates.
"send_goal map 0.0 0.0 0.0" will send the robot to a coordinate on the map
"send_goal relative 1.0 0.0 0.0" will send the robot to point relative to itself. In this case one meter straight forward
"send_goal go" will send the robot around the course from waypoint to waypoint (midterm 1)
"send_goal reset" will send the robot to its inital pose
**/
int main(int argc, char** argv)
{
    ros::init(argc,argv,"send_goals");
    ros::NodeHandle nh;

    ROS_INFO("Argc:%d",argc);

    //first set the initial pose for amcl
    ros::Publisher iniPose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1000);


    struct waypoint
    {
        float x;
        float y;
        float t;
    } waypoints;

    int laps=atof(argv[1]);
    if(laps==0)
    {
        laps=100;
    }
    waypoints.x = 2.80013;
    waypoints.y = 13.5616;
    waypoints.t = -110.0;

    MoveBaseClient ac("move_base",true);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    for(int j=0; j<laps; j++)
    {
        //go to the other waypoints
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoints.x;
        goal.target_pose.pose.position.y = waypoints.y;

        // Convert the Euler angle to quaternion
        double radians = waypoints.t * (M_PI/180);

        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);
        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(quaternion, qMsg);

        goal.target_pose.pose.orientation = qMsg;

//          ROS_INFO("Going to waypoint %d",i);
//          ROS_INFO("x=%f y=%f theta=%f",waypoints[i].x,waypoints[i].y,waypoints[i].t);
        ac.sendGoal(goal);
        // Wait for the action to return
        ac.waitForResult();

 //       if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //        ROS_INFO("You have reached the waypoint %d!",i);
   //     else
    //        ROS_INFO("The base failed for some reason");
    }

}
