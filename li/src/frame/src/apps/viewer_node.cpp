/*
Description: 
a. 功能:根据优化后的位姿生成点云地图；显示点云地图和当前帧点云
b. 输入：优化后的历史帧位姿；当前帧点云;当前帧位姿
c. 输出:按优化后的位姿投影之后的当前帧点云;按优化后的位姿投影之后的当前帧位姿;局部小地图点云;全局大地图点云
Author     : Wang Junpeng
data       : 
*/
#include <ros/ros.h>
#include "glog/logging.h"

#include <frame/saveMap.h>
#include "frame/global_defination/global_defination.h"
#include "frame/mapping/viewer/viewer_flow.hpp"

using namespace frame;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(saveMap::Request & request, saveMap::Response &response) {
  _need_save_map = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "viewer_node");
  ros::NodeHandle nh;
  
  ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
  std::shared_ptr<ViewerFlow> _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);

  ros::Rate rate(100);
  while(ros::ok()) {
    ros::spinOnce();

    _viewer_flow_ptr->Run();
    if(_need_save_map) {
      _need_save_map = false;
      _viewer_flow_ptr->SaveMap();
    }
    rate.sleep();
  }

  return 0;
}