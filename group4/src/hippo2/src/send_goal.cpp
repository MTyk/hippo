#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

/**
Moves the robot to given coordinates.
"send_goal map 0.0 0.0 0.0" will send the robot to a coordinate on the map
"send_goal relative 1.0 0.0 0.0" will send the robot to point relative to itself. In this case one meter straight forward
"send_goal go" will send the robot around the course from waypoint to waypoint (midterm 1)
"send_goal reset" will send the robot to its inital pose
**/
int main(int argc, char** argv){
  //if x,y,theta are set, move relative or to a point on the map
  if (argc=3){

  ros::init(argc,argv,"send_goals");
  ros::NodeHandle nh;

  double x=atof(argv[2]);
  double y=atof(argv[3]);
  double theta=atof(argv[4]);

  ROS_INFO("The goal is x=%f y=%f theta=%f",x,y,theta);

  MoveBaseClient ac("move_base",true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  if(argv[1]=="map"){
    goal.target_pose.header.frame_id = "map";   
  }else if(argv[1]=="relative"){
    goal.target_pose.header.frame_id = "base_link";
  }

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

  }else{
    //if a roundtrip is planned, execute the following (midterm 1)
    if(argv[1]=="go"){
      struct waypoint{
        float x;
        float y;
      } waypoints[7];

      //coordinates of the waypoints
      waypoints[0].x = 1.83652;
      waypoints[0].y = 12.4001;
      waypoints[1].x = 2.95875;
      waypoints[1].y = 12.2325;
      waypoints[2].x = 4.1000; 
      waypoints[2].y = 13.2758;
      waypoints[3].x = 4.03;
      waypoints[3].y = 14.3739;
      waypoints[4].x = 3.26894;
      waypoints[4].y = 15.3317;
      waypoints[5].x = 2.27449;
      waypoints[5].y = 14.5452;
      waypoints[6].x = 2.7223;
      waypoints[6].y = 13.54821;

      float theta=0.0;

      //go to first waypoint
      MoveBaseClient ac("move_base",true);

      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "map";   
      
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = waypoints[0].x;
      goal.target_pose.pose.position.y = waypoints[0].y;

        // Convert the Euler angle to quaternion
      double radians = theta * (M_PI/180);
      
      tf::Quaternion quaternion;
      quaternion = tf::createQuaternionFromYaw(radians);
      geometry_msgs::Quaternion qMsg;
      tf::quaternionTFToMsg(quaternion, qMsg);
      
      goal.target_pose.pose.orientation = qMsg;

      ROS_INFO("Going to waypoint 1");
      ac.sendGoal(goal);
      // Wait for the action to return
      ac.waitForResult();
      
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have reached the waypoint 1!");
      else
        ROS_INFO("The base failed for some reason");
      
      //go to the other waypoints
      for(int i=1;i<(sizeof(waypoints)/sizeof(*waypoints));i++){
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";   
      
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoints[i].x;
        goal.target_pose.pose.position.y = waypoints[i].y;

        // Convert the Euler angle to quaternion
        double radians = theta * (M_PI/180);
        
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(radians);
        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(quaternion, qMsg);
        
        goal.target_pose.pose.orientation = qMsg;
  
        ROS_INFO("Going to waypoint %d",i);
        ac.sendGoal(goal);
        // Wait for the action to return
        ac.waitForResult();
        
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          ROS_INFO("You have reached the waypoint %d!",i);
        else
          ROS_INFO("The base failed for some reason");
      }

    }else if(argv[1]=="reset"){
       //go to first waypoint
      MoveBaseClient ac("move_base",true);

      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "map";   
      
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = 1.14812;
      goal.target_pose.pose.position.y = 13.5645;
      float theta = -0.948;

        // Convert the Euler angle to quaternion
      double radians = theta * (M_PI/180);
      
      tf::Quaternion quaternion;
      quaternion = tf::createQuaternionFromYaw(radians);
      geometry_msgs::Quaternion qMsg;
      tf::quaternionTFToMsg(quaternion, qMsg);
      
      goal.target_pose.pose.orientation = qMsg;

      ROS_INFO("Going to initial pose!");
      ac.sendGoal(goal);
      // Wait for the action to return
      ac.waitForResult();
      
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have reached the initial pose!");
      else
        ROS_INFO("The base failed for some reason");
    }
  }
} 
