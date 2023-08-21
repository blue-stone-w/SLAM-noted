/*
Description: 点云滤波模块接口
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define FRAME_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "frame/sensor_data/cloud_data.hpp"

namespace frame {
  class CloudFilterInterface {
    public:
      virtual ~CloudFilterInterface() = default;

      virtual bool Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
  };
}
#endif