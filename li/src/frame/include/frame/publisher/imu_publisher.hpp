/*
Description: 在ROS中发布IMU数据
Author     : Wang Junpeng
data       : 
*/
#ifndef FRAME_PUBLISHER_IMU_PUBLISHER_HPP_
#define FRAME_PUBLISHER_IMU_PUBLISHER_HPP_

namespace frame {
  class IMUPublisher {
    public:
      IMUPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
      IMUPublisher() = default;
      void Publisher(IMUData imu_data);
      bool HasSubscribers();

    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      std::string frame_id_;
  };
}
#endif