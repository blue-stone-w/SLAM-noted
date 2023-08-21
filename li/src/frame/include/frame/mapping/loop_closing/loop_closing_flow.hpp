/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_
#define FRAME_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
#include "frame/subscriber/key_frame_subscriber.hpp"
#include "frame/publisher/loop_pose_publisher.hpp"
#include "frame/mapping/loop_closing/loop_closing.hpp"

namespace frame {
  class LoopClosingFlow {
    public:
      LoopClosingFlow(ros::NodeHandle& nh);
      bool Run();

    private:
      bool ReadData();
      bool HasData();
      bool ValidData();
      bool PublishData();

    private:
      std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
      std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;

      std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;

      std::shared_ptr<LoopClosing> loop_closing_ptr_;

      std::deque<KeyFrame> key_frame_buff_;
      std::deque<KeyFrame> key_gnss_buff_;

      KeyFrame current_key_frame_;
      KeyFrame current_key_gnss_;
  };
}
#endif