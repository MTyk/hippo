#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

#define TOLERANCE 0.5

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//function to check if 2 coordinate pairs are roughly the same
bool isSame(float x1, float y1, float xref, float yref){
  if( (x1 < (xref+TOLERANCE)) && (x1 > (xref-TOLERANCE)) && (y1 < (yref+TOLERANCE)) && (y1 > (yref-TOLERANCE))){
    return true;
  } else {
    return false;
  }
}

//Default Constructor
namespace global_planner
{

GlobalPlanner::GlobalPlanner()
{

}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
{

    plan.push_back(start);

    struct waypoint
    {
        float x;
        float y;
        float t;
    } waypoints[15];

    //coordinates of the waypoints
    waypoints[0].x = 1.78652;  //patch 1
    waypoints[0].y = 12.4001;
    waypoints[0].t = 0.0;
    waypoints[1].x = 2.08632; // pseudo-patch1
    waypoints[1].y = 12.1226;
    waypoints[1].t = 20.0;
    waypoints[2].x = 2.68124;
    waypoints[2].y = 11.9386;
    waypoints[2].t = 40.0;
    waypoints[3].x = 3.12151;   //patch 2
    waypoints[3].y = 12.1953; 
    waypoints[3].t = 60.0;
    waypoints[4].x = 3.39224;   
    waypoints[4].y = 12.4538; 
    waypoints[4].t = 60.0;
    waypoints[5].x = 3.6099;   
    waypoints[5].y = 12.6769; 
    waypoints[5].t = 60.0;
    waypoints[6].x = 4.29906; //patch 3
    waypoints[6].y = 13.3462;
    waypoints[6].t = 67.0;
    waypoints[7].x = 4.27455; 
    waypoints[7].y = 13.4848;
    waypoints[7].t = 67.0;
    waypoints[8].x = 4.27409;
    waypoints[8].y = 13.9606;
    waypoints[8].t = 67.0;
    waypoints[9].x = 4.13;  //patch 4
    waypoints[9].y = 14.4739;
    waypoints[9].t = 140.0;
    waypoints[10].x = 3.71302;  
    waypoints[10].y = 14.5837;
    waypoints[10].t = 140.0;
    waypoints[11].x = 3.52665;  
    waypoints[11].y = 14.8;
    waypoints[11].t = 140.0;
    waypoints[12].x = 3.26894; //patch 5
    waypoints[12].y = 15.3317;
    waypoints[12].t = -150.0;
    waypoints[13].x = 2.27449; //patch 6
    waypoints[13].y = 14.5452;
    waypoints[13].t = -60.0;
    waypoints[14].x = 2.80013; //patch 7
    waypoints[14].y = 13.5616;
    waypoints[14].t = -110.0; 

    geometry_msgs::PoseStamped new_goal = goal;
//    geometry_msgs::Quaternion qMsg;
    tf::Quaternion quaternion;


    if (isSame((float)start.pose.position.x, (float)start.pose.position.y , waypoints[1].x , waypoints[1].y)){
    
    for(int i = 1; i<0; i++)  //nothing done here i<0
    {
        double radians = waypoints[i].t * (M_PI/180);
        quaternion = tf::createQuaternionFromYaw(radians);
//        tf::quaternionTFToMsg(quaternion, qMsg);

//        new_goal.pose.orientation = qMsg;
      new_goal.pose.orientation.x = quaternion.x();
      new_goal.pose.orientation.y = quaternion.y();
      new_goal.pose.orientation.z = quaternion.z();
      new_goal.pose.orientation.w = quaternion.w();

        new_goal.pose.position.x = waypoints[i].x;
        new_goal.pose.position.y = waypoints[i].y;


        plan.push_back(new_goal);
    }
    } else if (isSame((float)start.pose.position.x, (float)start.pose.position.y, waypoints[6].x , waypoints[6].y)){
    
    for(int i = 7; i<12; i++)
    {
        double radians = waypoints[i].t * (M_PI/180);
        quaternion = tf::createQuaternionFromYaw(radians);
//        tf::quaternionTFToMsg(quaternion, qMsg);

//        new_goal.pose.orientation = qMsg;
      new_goal.pose.orientation.x = quaternion.x();
      new_goal.pose.orientation.y = quaternion.y();
      new_goal.pose.orientation.z = quaternion.z();
      new_goal.pose.orientation.w = quaternion.w();

        new_goal.pose.position.x = waypoints[i].x;
        new_goal.pose.position.y = waypoints[i].y;


        plan.push_back(new_goal);
    }
    } 
    plan.push_back(goal);
    return true;
}
};
