#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define TOYS_TOTAL 3

int main(int argc, char** argv)
{
    ros::init(argc,argv,"task_planner");
    ros::NodeHandle n;
    ros::ServiceClient pickupClient = n.serviceClient<std_srvs::Empty>("pickup");
    ros::ServiceClient dropClient = n.serviceClient<std_srvs::Empty>("drop");
    std_srvs::Empty srv;    //empty service to be passed in server calls

    //declare variables for navigation
    MoveBaseClient ac("move_base",true);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    tf::Quaternion quaternion;
    geometry_msgs::Quaternion qMsg;
    double radians;

    //State machine enum
    enum State {START, MOVE2TOY, PICKUP, MOVE2BASKET, DROP, END};
    State state = START;
  //  int nextState = 0;  //for testing only
    int toyIndex = 0;  //which toy to pursue now

      struct Goal{
        float x;
        float y;
        float t;	//orientation
      } toyLocation[TOYS_TOTAL], basketLocation;
    //TO DO: Define toy location array
      toyLocation[0].x = 1.3;
      toyLocation[0].y = 1.5;
      toyLocation[0].t = 0;
      toyLocation[1].x = 1.5;
      toyLocation[1].y = 0.6;
      toyLocation[1].t = 0;
      toyLocation[2].x = 1.8;
      toyLocation[2].y = -0.24;
      toyLocation[2].t = 0;
    //To DO: Define basket location
      basketLocation.x = 0.74;
      basketLocation.y = 0.27;
      basketLocation.t = 0;

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
                state = MOVE2TOY;
                break;
            case MOVE2TOY:
		//Send toyLocation[i] as goal to nav.
		            goal.target_pose.header.stamp = ros::Time::now();
       		      goal.target_pose.pose.position.x = toyLocation[toyIndex].x;
     		        goal.target_pose.pose.position.y = toyLocation[toyIndex].y;
        	      radians = toyLocation[toyIndex].t * (M_PI/180);	   // Convert the Euler angle to quaternion
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

                   // printf("toy index: %d\n", toyIndex);  //debugging
                }else{
		                 ROS_INFO("Failed to call pickup service.");
		            }
                break;
            case MOVE2BASKET:
		            // Go to basket
		            goal.target_pose.header.stamp = ros::Time::now();
       		      goal.target_pose.pose.position.x = basketLocation.x;
     		        goal.target_pose.pose.position.y = basketLocation.y;
        	      radians = basketLocation.t * (M_PI/180);	       	// Convert the Euler angle to quaternion
        	      quaternion = tf::createQuaternionFromYaw(radians);
        	      tf::quaternionTFToMsg(quaternion, qMsg);
        	      goal.target_pose.pose.orientation = qMsg;
		            ac.sendGoal(goal);
		            ac.waitForResult();
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){  //if basket reached
                    state = PICKUP;
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
                if(0){	//never
                    state = START;
                } else {
		                return 0;
		            }
                break;
        }

    }
    return 0;
}
