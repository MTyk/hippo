#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hippo2/Int.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalID.h"

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher velocityPublisher;
ros::Publisher cancelPublisher;

#define NUM_TOYS 8        //number of toys
#define NUM_VANTAGE 2       // number of vantage points
#define TOY_DIST_OFFSET 0.35   //in meters: used to offset toy location before sending coordinates to navigation so that the robot stops before it.
#define TIME_COUNT 6        //
#define SLEEP_TIME 0.5
#define RECOVER_TIME 2

struct Goal
{
    float x;
    float y;
    float s;  //orientation
} toyLocation[NUM_TOYS];    //relative to robot

int toyIndex;  //which toy to pursue now
int toyType = 0;
bool baseMoving = false;

void recover()
{
    actionlib_msgs::GoalID goalId;
    cancelPublisher.publish(goalId);
    ros::Duration(RECOVER_TIME).sleep();
    geometry_msgs::Twist msg;
    msg.linear.x=-0.1;
    msg.linear.y=0;
    msg.linear.z=0;
    msg.angular.x=0;
    msg.angular.y=0;
    msg.angular.z=0;
    velocityPublisher.publish(msg);
    ros::Duration(RECOVER_TIME).sleep();
    msg.linear.x=0.0;
    velocityPublisher.publish(msg);
}
void getToyLocation(const geometry_msgs::Vector3 msg)
{
    toyLocation[toyIndex].x = (msg.x-TOY_DIST_OFFSET)*cos(msg.y);
    toyLocation[toyIndex].y = (msg.x-TOY_DIST_OFFSET)*sin(msg.y);
    toyLocation[toyIndex].s = msg.y;
    ROS_INFO("Relative toy location: x=%f, y=%f, theta=%f", toyLocation[toyIndex].x, toyLocation[toyIndex].y, toyLocation[toyIndex].s);
}
void getVelocity(const geometry_msgs::Twist msg)
{
    ROS_INFO("Linear velocities: %f, %f",msg.linear.x,msg.angular.z);
    if( (msg.linear.x != 0.0) || (msg.angular.z != 0.0) )
    {
        baseMoving = true;
    }
    else
    {
        baseMoving = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"task_planner");
    ros::NodeHandle n;
    ros::ServiceClient pickupClient = n.serviceClient<std_srvs::Empty>("pickup");
    ros::ServiceClient dropClient = n.serviceClient<std_srvs::Empty>("drop");
    ros::ServiceClient locateToyClient = n.serviceClient<hippo2::Int>("locate_toy");
    ros::ServiceClient idToyClient = n.serviceClient<hippo2::Int>("id_toy");
    ros::ServiceClient halfLowerClient = n.serviceClient<std_srvs::Empty>("half_lower_gripper");
//   ros::ServiceClient initGripperClient = n.serviceClient<std_srvs::Empty>("init_gripper");
    std_srvs::Empty emptySrv;    //empty service to be passed in server calls
    hippo2::Int intSrv;     //Int service to pass in server calls

    ros::Subscriber toyLocSubscriber = n.subscribe("/toy_loc", 1, getToyLocation);
    ros::Subscriber velocitySubscriber = n.subscribe("/RosAria/cmd_vel", 1, getVelocity);
    velocityPublisher = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1000);
    cancelPublisher = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",1000);

    //declare variables for navigation
    MoveBaseClient ac("move_base",true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    tf::Quaternion quaternion;
    geometry_msgs::Quaternion qMsg;
    double radians;

    //State machine enum
    enum State {START , LOCATE , MOVE2VANTAGE, MOVE2TOY , ID , PICKUP , MOVE2BASKET , DROP , END};
    State state = START;
    int nextState = 0;  //for testing only

    int vantageIndex;     //which vantage point to go to

    Goal basketLocation[2], vantagePoint[NUM_VANTAGE];
    //TODO: Define type1 (white) basket location
    basketLocation[0].x = 11.575;
    basketLocation[0].y = 11.718;
    basketLocation[0].s = -80;
    //To DO: Define type2 (blue) basket location
    basketLocation[1].x = 14.459;
    basketLocation[1].y = 12.272;
    basketLocation[1].s = 0;

    vantagePoint[0].x=11.498;
    vantagePoint[0].y=13.719;
    vantagePoint[0].s=-60;

    vantagePoint[1].x=14.169;
    vantagePoint[1].y=14.165;
    vantagePoint[1].s=-110;

    while (ros::ok())
    {
//       printf("input transition #:\n");  //testing
//      scanf("%d", &nextState);    //testing
        int timeCount=0;
        switch (state)
        {
        case START:
            ROS_INFO("current state: START");
            // Initialize
            ROS_INFO("Initializing FSM");
            toyIndex = 0;
            vantageIndex = 0;
            // if(initGripperClient.call(emptySrv)){
            state = LOCATE;
            //}else{
            //ROS_INFO("Failed to call init_gripper service.");
            //}
            break;
        case LOCATE:
            ROS_INFO("current state: LOCATE");
            ROS_INFO("Pursuing toy number %d.", toyIndex+1);
            if(locateToyClient.call(intSrv))
            {
                if(intSrv.response.x)
                {
                    ROS_INFO("Toy located.");
                    ros::Duration(0.5).sleep();
                    ros::spinOnce();    //to get newly published toy location
                    state = MOVE2TOY;
                }
                else
                {
                    ROS_INFO("No toy located.");
                    state = MOVE2VANTAGE;
                }
            }
            else
            {
                ROS_INFO("Failed to call locate_toy service.");
            }
            break;
        case MOVE2TOY:
            ROS_INFO("current state: MOVE2TOY");
            //Send toyLocation[i] as goal to nav.
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = toyLocation[toyIndex].x;
            goal.target_pose.pose.position.y = toyLocation[toyIndex].y;
            quaternion = tf::createQuaternionFromYaw(toyLocation[toyIndex].s);	    // Convert the Euler angle to quaternion
            tf::quaternionTFToMsg(quaternion, qMsg);
            goal.target_pose.pose.orientation = qMsg;
            ac.sendGoal(goal);
            ROS_INFO("Toy number %d location sent to nav.", toyIndex+1);
            ac.waitForResult();
            ROS_INFO("result retrieved.");

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)   //if toy reached
            {
                state = ID;
            }
            else
            {
                ROS_INFO("failed to reach toy number %d.", toyIndex+1);
                state = MOVE2VANTAGE;
            }
            break;
        case MOVE2VANTAGE:
            ROS_INFO("current state: MOVE2VANTAGE");
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = vantagePoint[vantageIndex].x;
            goal.target_pose.pose.position.y = vantagePoint[vantageIndex].y;
            radians = vantagePoint[vantageIndex].s * (M_PI/180);     // Convert the Euler angle to quaternion
            quaternion = tf::createQuaternionFromYaw(radians);
            tf::quaternionTFToMsg(quaternion, qMsg);
            goal.target_pose.pose.orientation = qMsg;
            ac.sendGoal(goal);
            ROS_INFO("Vantage point number %d goal sent to nav.", vantageIndex+1);
            //ac.waitForResult();
            timeCount=0;
            while (1)
            {
                ros::Duration(SLEEP_TIME).sleep();
                ros::spinOnce();    //get velocity status
                actionlib::SimpleClientGoalState acState = ac.getState();   //get nav state

                ROS_INFO("Nav state: %s",acState.getText().c_str());

                if (!baseMoving)
                {
                    timeCount++;
                }
                else
                {
                    timeCount=0;
                }

                if(acState == actionlib::SimpleClientGoalState::SUCCEEDED)   //if toy reached
                {
                    ROS_INFO("Reached vantage point number %d.", vantageIndex+1);
                    state = LOCATE;
                    vantageIndex++;
                    if(vantageIndex >= NUM_VANTAGE)
                    {
                        vantageIndex = 0;
                    }
                    break;
                }
                if(acState == actionlib::SimpleClientGoalState::ABORTED)
                {
                    ROS_INFO("Failed to reach vantage point number %d.", vantageIndex+1);
                    recover();
                    break;
                }
                if(timeCount == TIME_COUNT)     // if the base gets stuck after a certain time of standing still
                {
                    ROS_INFO("Nav stuck. Performing recovery maneuvers.");
                    recover();
                    break;
                }
            }
            /*                ROS_INFO("result retrieved.");
                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){  //if toy reached
                                state = LOCATE;
                                vantageIndex++;
                                if(vantageIndex >= NUM_VANTAGE){
                                    vantageIndex = 0;
                                }
                            }else{
                                ROS_INFO("failed to reach vantage point number %d.", vantageIndex+1);
                            }*/
            break;
        case ID:
            ROS_INFO("current state: ID");
            if(idToyClient.call(intSrv))
            {
                toyType = intSrv.response.x;
                ROS_INFO("Toy ID successful: Type %d (1 = white, 2 = blue)", toyType);
                if (toyType == 0)
                {
                    toyType = 1;
                }
            }
            else
            {
                ROS_INFO("Failed to call id_toy service. Default (white) toy type assumed.");
                toyType = 1;	//default type (white)
            }
            vantageIndex = toyType;   // the next vantage point to go to is the one neares to the basket
            state = PICKUP;
            break;
        case PICKUP:
            ROS_INFO("current state: PICKUP");
            if(pickupClient.call(emptySrv))   //call service and wait for reply
            {
                ROS_INFO("Pickup successful");
                state = MOVE2BASKET;
                // printf("toy index: %d\n", toyIndex);  //debugging
            }
            else
            {
                ROS_INFO("Failed to call pickup service.");
            }
            break;
        case MOVE2BASKET:
            ROS_INFO("current state: MOVE2BASKET");
            // Go to basket
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = basketLocation[toyType-1].x;
            goal.target_pose.pose.position.y = basketLocation[toyType-1].y;
            radians = basketLocation[toyType-1].s * (M_PI/180);         // Convert the Euler angle to quaternion
            quaternion = tf::createQuaternionFromYaw(radians);
            tf::quaternionTFToMsg(quaternion, qMsg);
            goal.target_pose.pose.orientation = qMsg;
            ac.sendGoal(goal);
            ROS_INFO("Basket location sent.");
            //ac.waitForResult();
            timeCount=0;
            while (1)
            {
                ros::Duration(SLEEP_TIME).sleep();
                ros::spinOnce();    //get velocity status
                actionlib::SimpleClientGoalState acState = ac.getState();   //get nav state

                ROS_INFO("Nav state: %s",acState.getText().c_str());

                if (!baseMoving)
                {
                    timeCount++;
                }
                else
                {
                    timeCount=0;
                }

                if(acState == actionlib::SimpleClientGoalState::SUCCEEDED)   //if toy reached
                {
                    ROS_INFO("Reached basket.");
                    state = DROP;
                    break;
                }
                if(acState == actionlib::SimpleClientGoalState::ABORTED)
                {
                    ROS_INFO("failed to reach basket.");
                    recover();
                    break;
                }
                if(timeCount == TIME_COUNT)     // if the base gets stuck after a certain time of standing still
                {
                    ROS_INFO("Nav stuck. Performing recovery maneuvers.");
                    recover();
                    break;
                }
            }
        /*                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){  //if basket reached
                            state = DROP;
                        }else{
                            ROS_INFO("failed to reach basket.");
                        }
                        break;*/
            break;
        case DROP:
            ROS_INFO("current state: DROP");
            if(dropClient.call(emptySrv))  //call service and wait for reply
            {
                ROS_INFO("Drop successful");
                goal.target_pose.header.frame_id = "base_link";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = -0.5;
                goal.target_pose.pose.position.y = 0;
                quaternion = tf::createQuaternionFromYaw(M_PI);	    // Convert the Euler angle to quaternion
                tf::quaternionTFToMsg(quaternion, qMsg);
                goal.target_pose.pose.orientation = qMsg;
                ac.sendGoal(goal);
                ROS_INFO("Doing a 180.");
                ac.waitForResult();
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)   //if toy reached
                {
                    state = ID;
                    if(halfLowerClient.call(emptySrv))   //call service and wait for reply
                    {
                        ROS_INFO("Half lower gripper successful");
                    }
                    else
                    {
                        ROS_INFO("Failed to call half_lower_gripper service.");
                    }

                }
                else
                {
                    ROS_INFO("Failed to rotate.");
                    recover();
                }
                toyIndex++;  //next toy to go to
                if(toyIndex<NUM_TOYS)
                {
                    state = LOCATE;
                }
                else
                {
                    state = END;
                }
            }
            else
            {
                ROS_INFO("Failed to call drop service.");
            }
            break;
        case END:
            ROS_INFO("Ending run.");
            if(0)   //never
            {
                state = START;
            }
            else
            {
                return 0;
            }
            break;
        }

    }
    return 0;
}

