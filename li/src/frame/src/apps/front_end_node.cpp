/*
Description: 前端里程计的node文件
Author     : Wang Junpeng
data       : 
*/

#include <ros/ros.h>
#include "glog/logging.h"

#include "frame/global_defination/global_defination.h"
#include "frame/mapping/front_end/front_end_flow.hpp"

using namespace frame;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "front_end_node");
  ros::NodeHandle nh;

  std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

  ros::Rate rate(100);
  while(ros::ok()) {
    ros::spinOnce();
    front_end_flow_ptr->Run();
    rate.sleep();
  }
}