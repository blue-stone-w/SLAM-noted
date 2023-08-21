/*
Description: 点云畸变补偿
Author     : Wang Junpeng
data       : 
*/

#include "frame/models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace frame {
  //确定扫描周期和速度
  void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
  }

  //所有的雷达点旋转到保证数据中第一个雷达点是x轴正方向，这样在遍历点云的时候直接求变换后点云的反正切角度就行。
  //或者不做这个变换，在遍历点云的时候，每个点的反正切值都减去第一个点的反正切，也应该会有一样的效果
  bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& intput_cloud_ptr,
                                     CloudData::CLOUD_PTR& output_cloud_ptr) {
    //输入点云赋值给原始点云
    CloudData::CLOUD_PTR  origin_cloud_ptr(new CloudData::CLOUD(*intput_cloud_ptr));
    //清空输出点云指针
    output_cloud_ptr.reset(new CloudData::CLOUD());

    float orientation_space = 2.0 * M_PI;//雷达的旋转总角度，此处为360度雷达
    float delete_space = 5.0 * M_PI / 180;//小于5度则删除该点
    //第一个点与原点的连线与x轴的方向的夹角
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    //AngleAxisf，旋转的角轴表示法，给定旋转角度和旋转轴，旋转角度以弧度表示
    Eigen::AngleAxisf t_v(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_v.matrix();//矩阵形式；第一帧方向 = x轴正方向 * 该矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();//逆旋转

    //（源点云，变换后的点云，变换矩阵）旋转该帧所有点，使第一个点为x轴正方向；x轴正方向角度为0
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

    //坐标系旋转rotate_matrix的逆，因此此处应为rotate_matrix，以补偿坐标系的旋转造成的影响
    //可以参考函数f(x)左右平移时，括号内的变化
    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    for(size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
      float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
      if(orientation < 0.0) //方向在0-2PI內
        orientation += 2.0 *M_PI;

      if(orientation < delete_space || 2.0*M_PI - orientation < delete_space)
        continue;

      //以每帧的中间时刻作为基准值去畸变（因为bag包中的数据，每帧的时间戳是启始时刻和终止时刻的中值）
      float real_time = fabs(orientation)/orientation_space*scan_period_ - scan_period_/2.0;
      Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                   origin_cloud_ptr->points[point_index].y,
                                   origin_cloud_ptr->points[point_index].z);
      Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);//得到需要的旋转矩阵
      Eigen::Vector3f rotated_point = current_matrix * origin_point;//去除旋转畸变
      Eigen::Vector3f adjusted_point  = rotated_point + velocity_*real_time;//去除平移畸变
      CloudData::POINT point;
      point.x = adjusted_point(0);
      point.y = adjusted_point(1);
      point.z = adjusted_point(2);
      output_cloud_ptr->points.push_back(point);
    }

    //（源点云，变换后的点云，变换矩阵）旋转将该帧所有点，使第一个点旋转至原方向
    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
  }

  //旋转该点
  Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_*real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;//总的旋转
    return t_V.matrix();
  }
}