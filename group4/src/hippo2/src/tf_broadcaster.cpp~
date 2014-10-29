// Transform broadcaster - publishes the translation from base to laser
// see also http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF 

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  
  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        // Transform below (format: rotation, translation, time, parent, child)
        // TODO turn numbers into parameters
        // Yaw, pitch, roll = 0 (laser doesnt turn)
        // Laser offset 10cm (x), 20cm (z) TODO: measure correct vals
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.4)),
        ros::Time::now(),"base_link", "laser"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        // Transform below (format: rotation, translation, time, parent, child)
        // TODO turn numbers into parameters
        // Yaw, pitch, roll = 0 (laser doesnt turn)
        // Laser offset 10cm (x), 20cm (z) TODO: measure correct vals
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"odom", "base_link"));
    r.sleep();
  }
}
