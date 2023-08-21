/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_SENSOR_DATA_GNSS_DATA_HPP_
#define FRAME_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include "Geocentric/LocalCartesian.hpp"//在文件夹third_party中，没有复现

namespace frame {
  class GNSSData {
    public:
      double time = 0.0;
      double longitude = 0.0;//经
      double latitude = 0.0;//纬
      double altitude = 0.0;//海拔or海拔
      double local_E = 0.0;
      double local_N = 0.0;
      double local_U = 0.0;
      int status = 0;
      int service =0;

    private:
      static GeographicLib::LocalCartesian geo_converter;//Cartesian：笛卡尔
      static bool origin_position_inited;

    public:
      void InitOriginPosition();
      void UpdataXYZ();
      static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
  };
}

#endif