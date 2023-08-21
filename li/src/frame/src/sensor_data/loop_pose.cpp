/*
Description: 关键帧之间的相对位姿，用于闭环检测
Author     : Wang Junpeng
data       : 
*/

#include "frame/sensor_data/loop_pose.hpp"

namespace frame {
  Eigen::Quaternionf LoopPose::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
  }
}