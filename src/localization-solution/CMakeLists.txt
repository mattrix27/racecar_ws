cmake_minimum_required(VERSION 3.1)

project(localization-solution)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  sensor_msgs
  std_msgs
  tf
)
