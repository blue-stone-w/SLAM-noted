/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#include <ros/ros.h>
#include "glog/logging.h"

#include "frame/global_defination/global_defination.h"
#include "frame/mapping/loop_closing/loop_closing_flow.hpp"

using namespace frame;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "loop_closing_node");
  ros::NodeHandle nh;
  std::cout<<"loop node is right here"<<std::endl;
  std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);
  std::cout<<"LoopClosingFlow has been constructed"<<std::endl;

  ros::Rate rate(100);
  while(ros::ok()) {
    ros::spinOnce();
    loop_closing_flow_ptr->Run();
    rate.sleep();
  }

  return 0;
}