/*
Description: odemetry 信息发布
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define FRAME_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace frame {
  class OdometryPublisher {
    public:
      OdometryPublisher(ros::NodeHandle& nh,
                        std::string topic_name,
                        std::string base_frame_id,
                        std::string child_frame_id,
                        int buff_size);
      OdometryPublisher() = default;

      void Publish(const Eigen::Matrix4f& transform_matrix, double time);
      void Publish(const Eigen::Matrix4f& transform_matrix);

      bool HasSubscriber();

    private:
     void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      nav_msgs::Odometry odometry_;
  };
}
#endif