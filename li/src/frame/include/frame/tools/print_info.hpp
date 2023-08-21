/*
Description: 打印信息
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_TOOLS_PRINT_INFO_HPP_
#define FRAME_TOOLS_PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace frame {
  class PrintInfo {
    public:
      static void PrintPose(std::string head, Eigen::Matrix4f pose);
  };
}
#endif