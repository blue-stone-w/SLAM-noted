/*
Description: key frames信息发布
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define FRAME_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "frame/sensor_data/key_frame.hpp"

namespace frame {
  class KeyFramesPublisher {
    public:
    KeyFramesPublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string frame_id,
                      int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame>& key_frame);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
  };
}
#endif