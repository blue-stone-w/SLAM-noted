/*
Description: 订阅 key frame 数据
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_
#define FRAME_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "frame/sensor_data/key_frame.hpp"

namespace frame {
  class KeyFramesSubscriber {
    public:
      KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
      KeyFramesSubscriber() = default;
      void ParseData(std::deque<KeyFrame>& deque_key_frames);

    private:
      void msg_callback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);

    private:
      ros::NodeHandle nh_;
      ros::Subscriber subscriber_;
      std::deque<KeyFrame> new_key_frames_;
  };
}
#endif