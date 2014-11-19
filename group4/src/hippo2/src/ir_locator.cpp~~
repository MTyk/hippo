#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
 
 
#define RATE 10
 
float ir_range;
bool lift_down;
 
ros::Publisher pose_error;
ros::Publisher gripper_pub;
ros::Subscriber ar_sub;
ros::Subscriber irtest_sub;
 
geometry_msgs::Twist error;
 
ros::Time timer_start_time;
ros::Timer error_timer;
 
enum State{
    START,ANGULAR, LINEAR, ACTION
  };
State state=START; 
 
void irtest(const geometry_msgs::Vector3 msg){
  ir_range=msg.x;
}
 
void sendAngError(const ros::TimerEvent& timerevent){
  ros::Time time_called=timerevent.current_real;
 
  uint32_t timer_cnt=time_called.toNSec()-timer_start_time.toNSec();
  
  float ang_error=0.0;
  if(timer_cnt<=1000000000){
    ang_error=0.5;
  }else if(timer_cnt>1000000000&&timer_cnt<=3000000000){
    ang_error=-0.5;
  }else if(timer_cnt>2000000000){
    timer_start_time=timerevent.current_real;
  }
  if(ir_range>30.0){
    error.linear.x=0.0;
    error.linear.z=0.0;
    error.angular.x=ang_error;
    error.angular.z=0.1;
   
    pose_error.publish(error);
 
  }else{
    ROS_INFO("*** Object found, going into Linear mode");
    state=LINEAR;
    error.linear.x=0.0;
    error.linear.z=0.0;
    error.angular.x=0.0;
    error.angular.z=0.0;
    
    error_timer.stop();
    pose_error.publish(error);
  }
}
 
void setArduinoData(const geometry_msgs::Vector3 msg){
  //TODO check if right messages are received
  if(msg.x==7.0){
      if(msg.y==2.0){
        if(msg.z==1.0){
        lift_down=true;
        ROS_INFO("*** LIFT IS DOWN! ***");
        }else if(msg.z==0.0){
        lift_down=false;
        }
      } 
  }
  if(msg.x==8.0){
    ir_range=msg.y;
  }
  ROS_INFO("IR range: [%f]", ir_range);
}
 
bool pickup(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("service pickup called");
    ros::NodeHandle n;
    error_timer=n.createTimer(ros::Duration(0.1),sendAngError);
    bool isClockwise=true;
    ros::Rate loop_rate(RATE);
    //TODO Ros is not realtime, check how much it turns
    while (ros::ok()){
      ROS_INFO("%f",ir_range);
      switch(state){
        case START:
          state=ANGULAR;
          break;
        case ANGULAR:
          {
          timer_start_time=timer_start_time.now();
          error_timer.start();
          }
          break;
        case LINEAR:
          {
            ROS_INFO("Reached Linear mode");
            
            float lin_error=ir_range-15.0;
            float lin_tolerance=2.0;
 
            error.linear.x=lin_error*0.01;
            error.linear.z=lin_tolerance*0.01;
            error.angular.x=0.0;
            error.angular.z=0.0;
            
            ROS_INFO("*** Linear Error= %f cm ***",lin_error);
 
            if(lin_error>lin_tolerance){
              pose_error.publish(error);
            }else{
              error.linear.x=0.0;
              error.linear.z=0.0;
              error.angular.x=0.0;
              error.angular.z=0.0;
              pose_error.publish(error);
              ROS_INFO("*** Going to ACtion mode***"); 
              state=ACTION;
            }
          }
          break; 
        case ACTION:
          geometry_msgs::Vector3 gripper_cmd;
          
          gripper_cmd.x=1.0;
          gripper_cmd.y=255.0;
          gripper_cmd.z=300.0;
          gripper_pub.publish(gripper_cmd);
 
          ros::Duration(0.1).sleep();
          
          if(!lift_down){
          ROS_INFO("Lift down");
          gripper_cmd.x=2.0;
          gripper_cmd.y=-10.0;
          gripper_cmd.z=0.0;
          gripper_pub.publish(gripper_cmd);
          break;
          }
          ros::Duration(0.1).sleep();
          ROS_INFO("Catch");
          gripper_cmd.x=1.0;
          gripper_cmd.y=0.0;
          gripper_cmd.z=300.0;
          gripper_pub.publish(gripper_cmd);
          //Wait 0.1 sec so gripper is closed before next publish
          ros::Duration d = ros::Duration(0.1, 0);
          d.sleep();
 
          ros::Duration(0.1).sleep();
          
          ROS_INFO("Lift up");
          gripper_cmd.x=2.0;
          gripper_cmd.y=10.0;
          gripper_cmd.z=0.0;
          gripper_pub.publish(gripper_cmd);
 
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
    ros::NodeHandle n;
    error_timer=n.createTimer(ros::Duration(0.1),sendAngError);
    bool isClockwise=true;
    ros::Rate loop_rate(RATE);
    geometry_msgs::Twist error;
    //TODO Ros is not realtime, check how much it turns
    while (ros::ok()){
 
      switch(state){
        case START:
          state=ANGULAR;
          break;
        case ANGULAR:
          {
          timer_start_time=timer_start_time.now();
          error_timer.start();
          }
          break;
        case LINEAR:
          {
            error_timer.stop();
            float lin_error=ir_range-2.0;
            float lin_tolerance=1.0;
 
               error.linear.x=lin_error*0.01;
              error.linear.z=lin_tolerance*0.01;
              error.angular.x=0.0;
              error.angular.z=0.0;
              
            if(lin_error>lin_tolerance){

              pose_error.publish(error);
            }else{
              error.linear.x=0.0;
              error.linear.z=0.0;
              error.angular.x=0.0;
              error.angular.z=0.0;
              pose_error.publish(error);            
              state=ACTION;
            }
          }
          break; 
        case ACTION:
          geometry_msgs::Vector3 gripper_cmd;
          //TODO right values for publish?
          ROS_INFO("Drop");
          gripper_cmd.x=1.0;
          gripper_cmd.y=255.0;
          gripper_cmd.z=300.0;
          gripper_pub.publish(gripper_cmd);
 
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
 
  
  pose_error= n.advertise<geometry_msgs::Twist>("/pose_error", 1);
  gripper_pub = n.advertise<geometry_msgs::Vector3>("/arduino_command",10);
  ar_sub = n.subscribe("/arduino_data", 10, setArduinoData);
  irtest_sub = n.subscribe("/irtest",1000, irtest);
 
  ros::ServiceServer pickupService = n.advertiseService("pickup", pickup);
  ros::ServiceServer dropService = n.advertiseService("drop", drop); 
 
  ros::spin();
 
  return 0;
}
