/*
Description: 
Author     : Wang Junpeng
data       : 
*/
#include "frame/sensor_data/pose_data.hpp"

namespace frame {
  Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
  }
}
