/*
Description: 点云匹配模块的基类
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_MODELS_REGISTRATION_INTERFACE_HPP_
#define FRAME_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "frame/sensor_data/cloud_data.hpp"

namespace frame {
  class RegistrationInterface {
    public:
      virtual ~RegistrationInterface() = default;

      virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
      virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                             const Eigen::Matrix4f& predict_pose,
                             CloudData::CLOUD_PTR& result_cloud_ptr,
                             Eigen::Matrix4f& result_pose) = 0;
      virtual float GetFitnessScore() = 0;
  };
}
#endif