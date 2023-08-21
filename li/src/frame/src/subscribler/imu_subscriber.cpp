/*
Description: 订阅imu数据
*/

#include "frame/subscriber/imu_subscriber.hpp"
#include "glog/logging.h"

namespace frame {
  IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
      subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
    }

  void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    //已经定义了需要订阅的话题，该话题每发布新的消息，该函数就接收到消息
    //IMU消息的类型为sensor_msgs::ImuConstPtr，已经在ros中定义，参见本文件对应的h文件
    //imu_msg_ptr为指向IMU消息的常指针，指向新得到的IMU消息，旧的IMU消息被覆盖
    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();//ros提供的时间函数

    imu_data.linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    imu_data.angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;
    imu_data.orientation.w = imu_msg_ptr->orientation.w;

    new_imu_data_.push_back(imu_data);//把新的imu数据放入容器，容器类型为IMUData
  }

  void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {
    if(new_imu_data_.size() > 0) {
      //把新接收的imu数据全部放入imu_data_buff中，后续使用
      imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
      new_imu_data_.clear();
    }
  }
}