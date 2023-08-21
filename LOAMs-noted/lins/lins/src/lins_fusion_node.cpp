/*
Description: 
Author     : Wang Junpeng
date       : 
*/

#include <ros/ros.h>

#include <parameters.h>
#include <Estimator.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lins_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("\033[1;32m---->\033[0m LINS Fusion Started.");

  parameter::readParameters(pnh);

  fusion::LinsFusion lins(nh, pnh); // defined in Estimator.h
  lins.run(); // defined in Estimator.h

  ros::spin();
  return 0;
}