cmake_minimum_required(VERSION 2.6.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_executable(publisher publisher.cc)

add_executable(state_publisher src/talker.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})
