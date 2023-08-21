/*
Description: 订阅velocity数据
*/

#ifndef FRAME_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define FRAME_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "frame/sensor_data/velocity_data.hpp"

namespace frame {
  class VelocitySubscriber {
    public:
      VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
      VelocitySubscriber() = default;
      void ParseData(std::deque<VelocityData>& deque_velocity_data);

    private:
      void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

    private:
      ros::NodeHandle nh_;
      ros::Subscriber subscriber_;
      
      std::deque<VelocityData> new_velocity_data_;
  };
}

#endif