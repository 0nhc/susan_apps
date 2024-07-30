cmake_minimum_required(VERSION 3.0.2)
project(susan_apps)

find_package(catkin REQUIRED COMPONENTS
  rospy
  control_msgs
  sensor_msgs
  trajectory_msgs
  actionlib
)

catkin_package()