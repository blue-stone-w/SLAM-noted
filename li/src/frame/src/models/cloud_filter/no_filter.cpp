/*
Description: 不滤波
Author     : Wang Junpeng
data       : 
*/

#include "frame/models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace frame {
  NoFilter::NoFilter() {
  }

  bool NoFilter::Filter(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
  }
}