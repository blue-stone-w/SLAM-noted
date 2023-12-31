/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#include <ros/ros.h>
#include "glog/logging.h"

#include <frame/optimizeMap.h>
#include "frame/global_defination/global_defination.h"
#include "frame/mapping/back_end/back_end_flow.hpp"

using namespace frame;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request &Request, optimizeMap::Response &response) {
  _need_optimize_map = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "back_end_node");
  ros::NodeHandle nh;

  //节点调用服务(服务的名称,服务调用的函数)
  ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);
  _back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

  ros::Rate rate(100);
  while(ros::ok()) {
    ros::spinOnce();

    _back_end_flow_ptr->Run();

    if(_need_optimize_map) {
      _back_end_flow_ptr->ForceOptimize();
      _need_optimize_map = false;
    }
    rate.sleep();
  }

  return 0;
}