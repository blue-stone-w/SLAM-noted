/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define FRAME_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "frame/sensor_data/pose_data.hpp"

namespace frame {
  class OdometrySubscriber {
    public:
      OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
      OdometrySubscriber() = default;
      void ParseData(std::deque<PoseData>& deque_pose_data);

    private:
      void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

    private:
      ros::NodeHandle nh_;
      ros::Subscriber subscriber_;

      std::deque<PoseData> new_pose_data_;
  };
}
#endif