/*This node is intended for use by the Machine Vision node for visual motion control.
However it is general-purpose enough to be used by any other node should the need arise. */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define LOOP_RATE 10
#define MAX_SPEED 0.2

double linError = 0;
double angError = 0;
double linErrorTolerance = 0.00;
double angErrorTolerance = 0.00;

void getError(const geometry_msgs::Twist& e){
    linError = e.linear.x;
    angError = e.angular.x;
    linErrorTolerance = e.linear.z;
    angErrorTolerance = e.angular.z;
    ROS_INFO("linErr0:%f ",e.linear.x);
    ROS_INFO("linErr:%f ",linError);
    ROS_INFO("angErr:%f ",angError);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pose_adjuster");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/pose_error", 1, getError);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
    ros::Rate loop_rate(LOOP_RATE);

    geometry_msgs::Twist cmd;

    double pAction = 0;
    double iAction = 0;
    double dAction = 0;
    double linErrorLast = 0;
    double angErrorLast = 0;
    double linErrorSum = 0;
    double angErrorSum = 0;
    double linKp = 0.8;
    double linKi = 0;
    double linKd = 0.3;
    double angKp = 1.2;
    double angKi = 0;
    double angKd = 0.3;
    double sampleTime = double(1)/double(LOOP_RATE);

    while (ros::ok()){
        //Control angular orientation then position
        if ( (angError<0 && angError<angErrorTolerance*(-1)) || (angError>0 && angError>angErrorTolerance) ){ //abs(err)>tolerance?
            //If there is orientation error larger than the tolerance threshold either in +ve or -ve direction
            //control only angular velocity
            pAction = angError*angKp;
            dAction = angKd * (angError - angErrorLast) / sampleTime;
            angErrorSum = angErrorSum + angError;
            iAction = angKi * sampleTime * angErrorSum / 2;
            cmd.angular.z = pAction + dAction + iAction;
            angErrorLast = angError;

            //Set linear velocity to 0 to ensure only orientation is controlled
            cmd.linear.x = 0;
        } else if ( (linError<0 && linError<linErrorTolerance*(-1)) || (linError>0 && linError>linErrorTolerance) ){
            //Now control only linear velocity
            pAction = linError*linKp;
            ROS_INFO("p:%f ",pAction);
            dAction = linKd * (linError - linErrorLast) / sampleTime;
            ROS_INFO("d:%f ",dAction);
            ROS_INFO("linELast:%f ",linErrorLast);
            ROS_INFO("t:%f ",sampleTime);
            linErrorSum = linErrorSum + linError;
            iAction = linKi * sampleTime * linErrorSum / 2;
            cmd.linear.x = pAction + dAction + iAction;
            if (cmd.linear.x > MAX_SPEED){
              cmd.linear.x = MAX_SPEED;
            }
            linErrorLast = linError;
            //Set angular velocity to 0 to ensure only position is controlled
            ROS_INFO("lin cmd:%f ",cmd.linear.x);
            cmd.angular.z = 0;
        } else {    //if error is withing bounds do not move
            cmd.angular.z = 0;
            cmd.linear.x = 0;
        }

        //Dispatch message, get next error, then wait for the next cycle
        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }


}
