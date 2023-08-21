/*
  Description: 订阅激光点云信息，并解析数据
  Date:
*/

#ifndef FRAME_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define FRAME_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <deque>
//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>//在ros的include文件夹内，因此不需要表明文件路径

//pcl
#include <pcl/point_types.h>//在usr/include/pcl-1.7/pcl文件夹内
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "frame/sensor_data/cloud_data.hpp"

namespace frame {
  class CloudSubscriber {
    public:
      CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
      CloudSubscriber() = default;
      void ParseData(std::deque<CloudData>& deque_cloud_data);

    private:
      void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

    private:
      ros::NodeHandle nh_;
      ros::Subscriber subscriber_;
      std::deque<CloudData> new_cloud_data_;      
  };
}
#endif
