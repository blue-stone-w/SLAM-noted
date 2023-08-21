/*
Description: 打印信息
Author     : Wang Junpeng
data       : 
*/

#include "frame/tools/print_info.hpp"
#include "glog/logging.h"

namespace frame {
  void PrintInfo::PrintPose(std::string head, Eigen::Matrix4f pose) {
    Eigen::Affine3f aff_pose;//仿射变换矩阵
    aff_pose.matrix() = pose;
    float x, y, z, roll, pitch, yaw;
    // 从仿射变换矩阵提取得到各参数
    pcl::getTranslationAndEulerAngles(aff_pose, x, y, z, roll, pitch, yaw);
    std::cout << head
              << x << "," << y << "," << z << ","
              << roll*180/M_PI << "," << pitch*180/M_PI << "," << yaw*180/M_PI
              << std::endl;
  }
}
