/*
Description: 数据预处理模块，包括时间同步、点云去畸变等
Author     : Wang Junpeng
data       : 
*/

#include "frame/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "frame/global_defination/global_defination.h"

namespace frame {
  DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    //subscriber
    //将最早的数据纳入计算，并在使用后删除最早的数据
    //bag文件播放太快，来不及处理，会导致部分消息被直接舍弃
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "velo_link");

    //publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/velo_link", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "velo_link", 100);

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
  }

  bool DataPretreatFlow::Run() {
    if(!ReadData())
      return false;
    
    if(!InitCalibration())
      return false;

    if(!InitGNSS())
      return false;

    while(HasData()) {
      if(!ValidData())
        continue;

      TransformData();
      PublishData();
    }

    return true;
  }

  bool DataPretreatFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if(cloud_data_buff_.size() == 0)
      return false;

    double cloud_time = cloud_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    static bool sensor_inited = false;
    if((!sensor_inited)) {
      if(!valid_imu ||!valid_velocity || !valid_gnss) { //在这一点云时刻，三种数据中，至少有一种数据同步失败
        cloud_data_buff_.pop_front();//舍弃最早（最靠前）的点云
        return false;
      }
      sensor_inited = true;
    }

    return true;
  }

  bool DataPretreatFlow::InitCalibration() {
    static bool calibration_received = false;
    if(!calibration_received) {
      if(lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
        calibration_received = true;
      }
    }

    return calibration_received;
  }

  bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if(!gnss_inited) {
      GNSSData gnss_data = gnss_data_buff_.front();
      gnss_data.InitOriginPosition();
      gnss_inited = true;
    }

    return gnss_inited;
  }

  bool DataPretreatFlow::HasData() {
    if(cloud_data_buff_.size() == 0)
      return false;
    if(imu_data_buff_.size() == 0)
      return false;
    if(velocity_data_buff_.size() == 0)
      return false;
     if(gnss_data_buff_.size() == 0)
      return false;

    return true;
  }

  bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    //根据同步后的数据，同步时间不准确，则数据无效，舍弃最早的数据并返回
    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if(diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
      cloud_data_buff_.pop_front();//点云过早，舍弃点云数据，
      return false;
    }

    if(diff_imu_time > 0.05) {
      imu_data_buff_.pop_front();
      return false;
    }
    if(diff_velocity_time > 0.05) {
      velocity_data_buff_.pop_front();
      return false;
    }
    if(diff_gnss_time > 0.05) {
      gnss_data_buff_.pop_front();
      return false;
    }
    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
  }

  bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    //将gnss的经纬高转换为xyz(local_E,local_N,local_U)
    //Y轴正方向为正北
    current_gnss_data_.UpdataXYZ();
    //位置
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    //方向：由四元数得到欧几里的旋转矩阵
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= lidar_to_imu_;//转换至雷达坐标系，理论上应使用gnss到lidar的转换，此处作了简化
    
    current_velocity_data_.TransformCoordinate(lidar_to_imu_);//速度转换为imu系
    //设置扫描周期和速度
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
  }

  bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);

    return true;
  }
}