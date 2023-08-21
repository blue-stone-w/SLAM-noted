/*
Description: 关键帧的数据结构，在各个模块之间传递数据
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_SENSOR_DATA_KEY_FRAME_HPP_
#define FRAME_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace frame {
  class KeyFrame {
    public:
      double time = 0.0;
      unsigned int index = 0;
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    public:
    Eigen::Quaternionf GetQuaternion();
  };
}
#endif