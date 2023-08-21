/*
Description: 单个 关键帧key frame 的信息发布
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define FRAME_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "frame/sensor_data/key_frame.hpp"

namespace frame {
  class KeyFramePublisher {
    public:
      KeyFramePublisher(ros::NodeHandle& nh,
                        std::string topic_name,
                        std::string frame_id,
                        int buff_size);
      KeyFramePublisher() = default;
      void Publish(KeyFrame& key_frame);
      bool HasSubscribers();

    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      std::string frame_id_ = "";
  };
}
#endif