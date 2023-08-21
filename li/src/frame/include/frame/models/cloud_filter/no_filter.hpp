/*
Description: 不滤波
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define FRAME_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "frame/models/cloud_filter/cloud_filter_interface.hpp"

namespace frame {
  class NoFilter: public CloudFilterInterface {
    public:
      NoFilter();
      bool Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
  };
}
#endif