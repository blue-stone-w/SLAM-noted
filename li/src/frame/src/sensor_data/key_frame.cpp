/*
Description: 关键帧的数据结构，在各个模块之间传递数据
Author     : Wang Junpeng
data       : 
*/

#include "frame/sensor_data/key_frame.hpp"

namespace frame {
  Eigen::Quaternionf KeyFrame::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
  }
}