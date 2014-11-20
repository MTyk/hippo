#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
 
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
#define TOYS_TOTAL 5
 
int main(int argc, char** argv)
{
    ros::init(argc,argv,"task_planner");
    ros::NodeHandle n;
    ros::ServiceClient pickupClient = n.serviceClient<std_srvs::Empty>("pickup");
    ros::ServiceClient dropClient = n.serviceClient<std_srvs::Empty>("drop");
//   ros::ServiceClient initGripperClient = n.serviceClient<std_srvs::Empty>("init_gripper");
    std_srvs::Empty srv;    //empty service to be passed in server calls
 
    //declare variables for navigation
    MoveBaseClient ac("move_base",true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    tf::Quaternion quaternion;
   geometry_msgs::Quaternion qMsg;
    double radians;
 
    //State machine enum
    enum State {START, MOVE2TOY, PICKUP, MOVE2BASKET, DROP, END};
    State state = START;
  //  int nextState = 0;  //for testing only
    int toyIndex;  //which toy to pursue now
 
      struct Goal{
        float x;
        float y;
        float s;  //orientation
      } toyLocation[TOYS_TOTAL], basketLocation;
    //TO DO: Define toy location array
      toyLocation[0].x = 3.78;
      toyLocation[0].y = 13.61;
      toyLocation[0].s = 20;  //0 0 0.24851 0.96863
      toyLocation[1].x = 3.6865;
      toyLocation[1].y = 13.855;
      toyLocation[1].s = 50; //0 0 0.43659 0.89966
      toyLocation[2].x = 3.3752;
      toyLocation[2].y = 14.266;
      toyLocation[2].s = 10;//0 0 0.18856 0.98206
      toyLocation[3].x = 3.075;
      toyLocation[3].y = 14.587;
      toyLocation[3].s = 10;//0 0 0.84427 0.53591
      toyLocation[4].x = 2.9597;
      toyLocation[4].y = 14.73;
      toyLocation[4].s = 40;//0 0 0.80992 0.58654
    //To DO: Define basket location
      basketLocation.x = 3.4033;
      basketLocation.y = 12.831;
      basketLocation.s = -40;//0 0 -0.45266 0.89168
       
       
       
 
    while (ros::ok())
    {
        ROS_INFO("current state: %d", state);
       // printf("current state: %d\ninput transition #:\n", state);  //testing
      //  scanf("%d", &nextState);    //testing
        switch (state)
        {
            case START:
                // Initialize
                toyIndex = 0;
               // if(initGripperClient.call(srv)){
                  state = MOVE2TOY;
                //}else{
                  //ROS_INFO("Failed to call init_gripper service.");
                  //}
                break;
            case MOVE2TOY:
    //Send toyLocation[i] as goal to nav.
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = toyLocation[toyIndex].x;
                goal.target_pose.pose.position.y = toyLocation[toyIndex].y;
                radians = toyLocation[toyIndex].s * (M_PI/180);     // Convert the Euler angle to quaternion
                quaternion = tf::createQuaternionFromYaw(radians);
                tf::quaternionTFToMsg(quaternion, qMsg);
                goal.target_pose.pose.orientation = qMsg;
                ac.sendGoal(goal);
                ROS_INFO("goal sent.");
                ac.waitForResult();
                ROS_INFO("result retrieved.");
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){  //if toy reached
                    state = PICKUP;
                }else{
                    ROS_INFO("failed to reach toy number %d.", toyIndex+1);
                }
                break;
            case PICKUP:
                if(pickupClient.call(srv)){     //call service and wait for reply
                    state = MOVE2BASKET;
                   // printf("toy index: %d\n", toyIndex);  //debugging
                }else{
                     ROS_INFO("Failed to call pickup service.");
                }
                break;
            case MOVE2BASKET:
                // Go to basket
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = basketLocation.x;
                goal.target_pose.pose.position.y = basketLocation.y;
                radians = basketLocation.s * (M_PI/180);         // Convert the Euler angle to quaternion
                quaternion = tf::createQuaternionFromYaw(radians);
                tf::quaternionTFToMsg(quaternion, qMsg);
                goal.target_pose.pose.orientation = qMsg;
                ac.sendGoal(goal);
                ac.waitForResult();
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){  //if basket reached
                    state = DROP;
                }else{
                    ROS_INFO("failed to reach basket.");
                }
                break;
            case DROP:
                if(dropClient.call(srv)){ //call service and wait for reply
                    toyIndex++;     //next toy to go to
                    if(toyIndex<TOYS_TOTAL){
                        state = MOVE2TOY;
                    }else{
                        state = END;
                    }
                }else{
                    ROS_INFO("Failed to call drop service.");
                }
                break;
            case END:
                if(0){  //never
                    state = START;
                } else {
                    return 0;
                }
                break;
        }
 
    }
    return 0;
}
