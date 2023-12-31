/*
Description: 关键帧之间的相对位姿，用于闭环检测
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_SENSOR_DATA_LOOP_POSE_HPP_
#define FRAME_SENSOR_DATA_LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace frame {
  class LoopPose {
    public:
      double time = 0.0;
      unsigned int index0 = 0;
      unsigned int index1 = 0 ;
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    public:
      Eigen::Quaternionf GetQuaternion();
  };
}
#endif