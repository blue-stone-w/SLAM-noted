/*
Description: 前端里程计算法
Author     : Wang Junpeng
data       : 
*/
#include "frame/mapping/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "frame/global_defination/global_defination.h"
#include "frame/tools/print_info.hpp"
#include "frame/models/registration/ndt_registration.hpp"
#include "frame/models/cloud_filter/voxel_filter.hpp"
#include "frame/models/cloud_filter/no_filter.hpp"

namespace frame {
  FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()) {
      InitWithConfig();
    }

  bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);//node用于存储解析后的 yaml 信息。



    std::cout << "-----------------前端初始化-------------------" << std::endl;
    InitParam(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
  }

  bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
  }
  //找到对应的匹配方法，创建匹配算法的对象实例
  bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "前端选择的点云匹配方式：" <<  registration_method << std::endl;

    if(registration_method == "NDT") {
      registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
      LOG(ERROR) << "没找到与" << registration_method << "相对应的点云匹配方式！";
      return false;
    }
    //如果后续添加新的匹配算法，可以添加新的if语句，并将匹配指针指向新的匹配类

    return true;
  }
  //找到对应的滤波方法，创建滤波方法的对象实例
  bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "前端" << filter_user << "选择的滤波方法为：" << filter_method << std::endl;

    if(filter_method == "voxel_filter") {
      filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    }
    else if(filter_method == "no_filter") {
           filter_ptr = std::make_shared<NoFilter>();
         }
         else {
           LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_method << " 相对应的滤波方法!";
           return true;
         }
    return false;
  }

  bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    //cloud_pose对应的实参为front_end_flow中的laser_odometry_
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    //局部地图容器中没有关键帧，说明是第一帧数据
    //此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if(local_map_frames_.size() == 0) {
      current_frame_.pose = init_pose_;
      UpdateWithNewFrame(current_frame_);
      cloud_pose = current_frame_.pose;
      return true;
    }

    //不是第一帧，正常匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr,  current_frame_.pose);
    cloud_pose = current_frame_.pose;

    //更新相邻两帧的运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    //匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新(曼哈顿距离)
    if(fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) +
       fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
       fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
       UpdateWithNewFrame(current_frame_);
       last_key_frame_pose = current_frame_.pose;
    }

    return true;
  }

  bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
  }

  //用新的关键帧更新局部地图和ndt匹配的目标点云
  bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    //这一步的目的是把关键帧的点云保存下来
    //使用共享指针，所以复制指针即可
    //此时无论放多少关键帧在容器里，这些关键帧点云指针都指向同一帧点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    //更新局部地图
    local_map_frames_.push_back(key_frame);
    while(local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
      local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for(size_t i=0; i < local_map_frames_.size(); ++i) {
      pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,//cloud_in
                               *transformed_cloud_ptr,//cloud_out
                               local_map_frames_.at(i).pose);//transform matrix
      *local_map_ptr_ += *transformed_cloud_ptr;
    }

    //更新ndt匹配的目标点云
    //关键帧数量较少时不滤波，以防点云太稀疏影响匹配效果
    if(local_map_frames_.size() < 10) {
      registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
      CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
      local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
      registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    return true;
  }
}
