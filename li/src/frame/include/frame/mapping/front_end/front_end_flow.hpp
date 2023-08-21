/*
Description: 前端里程计(front end)任务管理
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define FRAME_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "frame/subscriber/cloud_subscriber.hpp"
#include "frame/publisher/odometry_publisher.hpp"
#include "frame/mapping/front_end/front_end.hpp"

namespace frame {
  class FrontEndFlow {
    public:
      FrontEndFlow(ros::NodeHandle& nh);
      bool Run();

    private:
      bool ReadData();
      bool HasData();
      bool ValidData();
      bool UpdateLaserOdometry();
      bool PublishData();

    private:
      std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
      std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
      std::shared_ptr<FrontEnd> front_end_ptr_;

      std::deque<CloudData> cloud_data_buff_;

      CloudData current_cloud_data_;

      Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
  };
}
#endif