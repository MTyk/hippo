#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
   
#define SLEEP 0.2 
#define RATE 10

#define DISTANCE 40.0
#define DISTANCEBASKET 15
#define SETPOINTTOY 15.0
#define SETPOINTBASKET 13.0
   
float ir_range;
   
ros::Publisher pose_error;
ros::Subscriber ar_sub;
 
ros::ServiceClient raiseGripperClient;
ros::ServiceClient lowerGripperClient;
ros::ServiceClient openGripperClient;
ros::ServiceClient closeGripperClient;
 
std_srvs::Empty srv;
//ros::Subscriber irtest_sub;
 
ros::Timer error_timer;

float lastError=100.0;
 
int ang_counter=0; 
 
 bool basket = false;
geometry_msgs::Twist error; 
   
enum State{
    START,ANGULAR, LINEAR, ACTION, ERROR
  };
State pickupState=ANGULAR;
State dropState=START;
   
/*void irtest(const geometry_msgs::Vector3 msg){
  ir_range=msg.x;
}*/
   
void sendAngError(const ros::TimerEvent& timerevent){
 
 ang_counter++;
  
  float ang_error=0.0;
   
  if(ang_counter<=10){
    ang_error=0.2;
  }else if(ang_counter>10 && ang_counter<=30){
    ang_error=-0.2;
  } else{
    ang_counter=0;
  }
    float dist;
    if(basket){
    dist = DISTANCEBASKET;
    }else{
    dist = DISTANCE;
    } 
  if(ir_range>dist){
    error.linear.x=0.0;
    error.linear.z=0.0;
    error.angular.x=ang_error;
    error.angular.z=0.1;
    pose_error.publish(error);
   
  }else{
    ROS_INFO("*** Object found, going into Linear mode");
    pickupState=LINEAR;
    dropState=LINEAR;
    error.linear.x=0.0;
    error.linear.z=0.0;
    error.angular.x=0.0;
    error.angular.z=0.0;
      
    ang_counter=0;
      
    error_timer.stop();
    pose_error.publish(error);
  }
}
   
void setArduinoData(const geometry_msgs::Vector3 msg){
 
  if(msg.x==8.0){
    ir_range=msg.y;
  }
 }
bool init_gripper(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
 
   
} 
bool pickup(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("service pickup called");
    basket=false;
    ros::NodeHandle n;
    error_timer=n.createTimer(ros::Duration(0.1),sendAngError);
    bool isClockwise=true;
    ros::Rate loop_rate(RATE);
   
    while (ros::ok()){
     // ROS_INFO("%f",ir_range);
      switch(pickupState){
        case START:
         /* if(raiseGripperClient.call(srv)){
            ROS_INFO("*** Gripper init raise ***");
          }
          if(openGripperClient.call(srv)){
            ROS_INFO("*** Gripper Init open *** ");
          }
          pickupState=ANGULAR;*/
          break;
        case ANGULAR:
          {
          ROS_INFO("*** Pickup angular");
          error_timer.start();
          }
          break;
        case LINEAR:
          {
            ROS_INFO("*** Pickup linear");
  
            if(ir_range>DISTANCE){
              ROS_INFO("*** Lost Object, going back to angular ***");
              pickupState=ANGULAR;
              break;
            }
              
            float lin_error=ir_range-SETPOINTTOY;
            float lin_tolerance=2.0;
            
            if(lin_error-lastError < 7.0){
   
            error.linear.x=lin_error*0.01;
            error.linear.z=lin_tolerance*0.01;
            error.angular.x=0.0;
            error.angular.z=0.0;
              
            ROS_INFO("*** Linear Error= %f cm ***",lin_error);
            }else{
                ROS_INFO("*** Lost Object, going back to angular ***");
                pickupState=ANGULAR;
                break;
            }
   
   
            if ( ((lin_error<0 && lin_error<lin_tolerance*(-1)) || (lin_error>0 && lin_error>lin_tolerance)) ){
              pose_error.publish(error);
              lastError=lin_error;
            }else{
              error.linear.x=0.0;
              error.linear.z=0.0;
              error.angular.x=0.0;
              error.angular.z=0.0;
              pose_error.publish(error);
              lastError=lin_error;
              ROS_INFO("*** Going to ACtion mode***");
              pickupState=ACTION;
            }
          }
          break;
        case ACTION:
          ROS_INFO("*** Pickup action ***");
          geometry_msgs::Vector3 gripper_cmd;
           
          if(!lowerGripperClient.call(srv)){
            return false;
          }
          if(!closeGripperClient.call(srv)){
            ROS_INFO("*** Closing Gripper failed ***");
            return false;
          }
 
          if(!raiseGripperClient.call(srv)){
            ROS_INFO("*** Raising Gripper failed ***");
            return false;
          }
          ROS_INFO("Before return");
          return true;
          break;     
      }
 
   
    ros::spinOnce();
    loop_rate.sleep();
    }
   
  return false;
}
   
bool drop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("service drop called");
    basket=true;
    lastError=100.0;
    ros::NodeHandle n;
    error_timer=n.createTimer(ros::Duration(0.1),sendAngError);
    bool isClockwise=true;
    ros::Rate loop_rate(RATE);
    geometry_msgs::Twist error;
    //TODO Ros is not realtime, check how much it turns
    while (ros::ok()){
   
      switch(dropState){
        case START:
          dropState=ANGULAR;
          break;
        case ANGULAR:
          {
          ROS_INFO("*** Drop Angular ***");
          error_timer.start();
          }
          break;
        case LINEAR:
          {
            ROS_INFO("*** Drop Linear ***");
  
            if(ir_range>DISTANCEBASKET+2.0){
              ROS_INFO("*** Lost Object, going back to angular ***");
              dropState=ANGULAR;
              break;
            }
  
            error_timer.stop();
            float lin_error=ir_range-SETPOINTBASKET;
            float lin_tolerance=2.0;
   
               error.linear.x=lin_error*0.01;
              error.linear.z=lin_tolerance*0.01;
              error.angular.x=0.0;
              error.angular.z=0.0;
                
            if ( ((lin_error<0 && lin_error<lin_tolerance*(-1)) || (lin_error>0 && lin_error>lin_tolerance)) ){
              pose_error.publish(error);
            }else{
              error.linear.x=0.0;
              error.linear.z=0.0;
              error.angular.x=0.0;
              error.angular.z=0.0;
              pose_error.publish(error);
              ROS_INFO("*** Going to ACtion mode***");
              dropState=ACTION;
            }
          }
          break;
        case ACTION:
          ROS_INFO("*** Drop Action ***");
          if(openGripperClient.call(srv)){
          }
          return true;
          break;
               
      }
   
    ros::spinOnce();
    loop_rate.sleep();
    }
   
  return false;
}
   
   
   
int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "ir_locator");
   
  ros::NodeHandle n;
   
    
  pose_error= n.advertise<geometry_msgs::Twist>("/pose_error", 10);
  ar_sub = n.subscribe("/arduino_data", 100, setArduinoData);
 // irtest_sub = n.subscribe("/irtest",1000, irtest);
   
  raiseGripperClient = n.serviceClient<std_srvs::Empty>("raise_gripper");
  lowerGripperClient = n.serviceClient<std_srvs::Empty>("lower_gripper");
  openGripperClient = n.serviceClient<std_srvs::Empty>("open_gripper");
  closeGripperClient = n.serviceClient<std_srvs::Empty>("close_gripper");
  ros::ServiceServer pickupService = n.advertiseService("pickup", pickup);
  ros::ServiceServer dropService = n.advertiseService("drop", drop);
  ros::ServiceServer init_gripper_service = n.advertiseService("init_gripper", init_gripper);
 
 
  ros::spin();
   
  return 0;
}
