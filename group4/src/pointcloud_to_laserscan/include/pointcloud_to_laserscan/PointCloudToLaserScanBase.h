/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#ifndef POINTCLOUD_TO_LASERSCAN_ROS
#define POINTCLOUD_TO_LASERSCAN_ROS

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <tf/transform_listener.h>

namespace pointcloud_to_laserscan
{ 
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
class PointCloudToLaserScanBase
{
public:
    

    PointCloudToLaserScanBase(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    
    ~PointCloudToLaserScanBase();

private:

    void cloudCb(const PointCloud::ConstPtr& cloud);

    void connectCb();

    void disconnectCb();
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    tf::TransformListener tf_;

    boost::mutex connect_mutex_;

    unsigned int input_queue_size_;
    std::string target_frame_;
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
    bool use_inf_;
};

} // pointcloud_to_laserscan

#endif
