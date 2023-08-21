/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_SENSOR_DATA_POSE_DATA_HPP_
#define FRAME_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace frame {
  class PoseData {
    public:
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
      double time = 0.0;

    public:
    Eigen::Quaternionf GetQuaternion();
  };
}
#endif