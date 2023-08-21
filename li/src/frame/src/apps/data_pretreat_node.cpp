/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#include <ros/ros.h>//不需要自己编写
#include <glog/logging.h>//谷歌的日志工具，不需要自己编写

#include "frame/global_defination/global_defination.h"//作用未知
#include "frame/data_pretreat/data_pretreat_flow.hpp"

using namespace frame;

int main(int argc, char *argv[]) {
	google::InitGoogleLogging(argv[0]);//初始化谷歌日志
	FLAGS_log_dir = WORK_SPACE_PATH + "/Log";//猜测WORK_SPACE_PATH为工作空间目录
	FLAGS_alsologtostderr = 1;//I guess: log to std error，也输出到标准错误函数
	ros::init(argc, argv, "data_pretreat_node");//初始化本节点
	ros::NodeHandle nh;//创建指向本节点的句柄

	//创建类型为DataPretreatFlow的data_pretreat_flow_ptr，用类型为DataPretreatFlow的值赋值
	std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);
	ros::Rate rate(100);
	while(ros::ok()) {
		ros::spinOnce();

		data_pretreat_flow_ptr->Run();

		rate.sleep();
	}
}
