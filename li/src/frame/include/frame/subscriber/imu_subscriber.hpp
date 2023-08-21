/*
Description: 订阅imu数据
Author     : Wang Junpeng
data       : 
*/


#ifndef FRAME_SUBSCRIBER_IMU_SUNSCRIBER_HPP_
#define FRAME_SUBSCRIBER_IMU_SUNSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "frame/sensor_data/imu_data.hpp"

namespace frame {
  class IMUSubscriber {
    public:
      //确定订阅的话题，以及缓存的消息的数量
      IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
      IMUSubscriber() = default;

      //按照node文件中定义的频率运行
      void ParseData(std::deque<IMUData>& deque_imu_data);

    private:
      //只要有该话题的消息出现，就调用该函数
      void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
      ros::NodeHandle nh_;
      ros::Subscriber subscriber_;

      //在node文件中限定了更新频率，在两次更新之间，新得到的消息保存在此处
      //当IMU数据被获取并转移至其它变量后，清空该变量
      std::deque<IMUData> new_imu_data_;
  };
}
#endif