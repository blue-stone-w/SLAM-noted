/*
Description: 
Author     : Wang Junpeng
data       : 
*/

#include "frame/sensor_data/gnss_data.hpp"

#include "glog/logging.h"

//静态成员变量必须在类外初始化,以使其在整个程序运行期间保持稳定
//否则每次使用该类创建的对象，都会重新给静态变量赋值，造成重复定义
//而且静态成员变量是与类关联的对象，不与类对象关联
bool frame::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian frame::GNSSData::geo_converter;

namespace frame {
  void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
  }

  void GNSSData::UpdataXYZ() {
    if(!origin_position_inited) {
      LOG(WARNING) << "GeoConverter has not set origin position";
    }
    //推测作用是将新的gnss数据转换为局部笛卡尔坐标系，并放入geo_converter中
    //Geocentric/LocalCartesian.hpp -> Geocentric.hpp
    //Forward -> IntForward -> Geocentric _earth -> Math::sincosd 后面的看不懂了
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
    //经纬高转换为xyz，是Y轴指向正北的局部笛卡尔坐标系 。
    //("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
  }
  
  bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    //传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    //即找到与同步时间相邻的左右两个数据
    //相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间相差太远不合适插值
    while(UnsyncedData.size() >= 2) {
      if (UnsyncedData.front().time > sync_time)
        return false;
       if(UnsyncedData.at(1).time < sync_time) {
         UnsyncedData.pop_front();
           continue;
       }
       if(sync_time - UnsyncedData.front().time > 0.2) {
         UnsyncedData.pop_front();
           break;
       }
       if(UnsyncedData.at(1).time - sync_time > 0.2) {
         UnsyncedData.pop_front();
         break;
       }
       break;//此时已经找到合适时间戳的数据
    }

    if(UnsyncedData.size() < 2)
      return false;

    //定义相邻数据以及同步后的数据
    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    //插值获得变量SyncedData的值
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);

    return true;
  }
}