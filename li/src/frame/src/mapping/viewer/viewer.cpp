/*
Description: 实时显示，包括点云
Author     : Wang Junpeng
data       : 
*/

#include "frame/mapping/viewer/viewer.hpp"

#include <pcl-1.7/pcl/common/transforms.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "frame/tools/file_manager.hpp"
#include "frame/global_defination/global_defination.h"

namespace frame {
  Viewer::Viewer() {
    InitWithConfig();
  }

  bool Viewer::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/viewer.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------显示模块初始化-------------------" << std::endl;
    InitParam(config_node);
    InitDataPath(config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("global_map", global_map_filter_ptr_, config_node);

    return true;
  }

  bool Viewer::InitParam(const YAML::Node& config_node) {
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;    
  }
  bool Viewer::InitDataPath(const  YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
      data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";
    map_path_ = data_path + "/slam_data/map";
    
    if(!FileManager::InitDirectory(map_path_, "点云地图文件"))
      return false;

    return true;
  }

  bool Viewer::InitFilter(std::string filter_user,
                          std::shared_ptr<CloudFilterInterface>& filter_ptr,
                          const YAML::Node& config_node) {
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "显示模块" << filter_user << "选择的滤波方法为：" << filter_method << std::endl;
    if(filter_method == "voxel_filter") {
      filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_method << " 相对应的滤波方法!";
        return false;
    }
    return true;
  }

  bool Viewer::UpdateWithOptimizedKeyFrames(std::deque<KeyFrame>& optimized_key_frames) {
    //优化后的关键帧构成全局地图
    has_new_global_map_ = false;
    if(optimized_key_frames.size() > 0) {
      optimized_key_frames_ = optimized_key_frames;
      optimized_key_frames.clear();
      OptimizeKeyFrames();
      has_new_global_map_ = true;
    }
    return has_new_global_map_;
  }

  bool Viewer::UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                                     PoseData transformed_data,
                                     CloudData cloud_data) {
    //新的关键帧构成局部地图
    has_new_local_map_ = false;

    //把新的关键帧存入容器all_key_frames_中
    if(new_key_frames.size() > 0) {
      KeyFrame key_frame;
      for(size_t i=0; i < new_key_frames.size(); ++i) {
        key_frame = new_key_frames.at(i);
        key_frame.pose = pose_to_optimize_ * key_frame.pose;
        all_key_frames_.push_back(key_frame);
      }
      new_key_frames.clear();
      has_new_local_map_ = true;
    }

    optimized_odom_ = transformed_data;
    optimized_odom_.pose = pose_to_optimize_ * optimized_odom_.pose;
    optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *optimized_cloud_.cloud_ptr, optimized_odom_.pose);

    return true;
  }

  bool Viewer::OptimizeKeyFrames() {
    size_t optimized_index = 0;
    size_t all_index = 0;
    while(optimized_index < optimized_key_frames_.size() && all_index < all_key_frames_.size()) {
      if(optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
        optimized_index++;
      } else if(optimized_key_frames_.at(optimized_index).index > all_key_frames_.at(all_index).index) {
        all_index++;
      } else {
        pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose * all_key_frames_.at(all_index).pose.inverse();
        all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);
        optimized_index++;
        all_index++;
      }
    }
    while(all_index < all_key_frames_.size()) {
      //按优化后的位姿投影之后的当前帧点云
      all_key_frames_.at(all_index).pose = pose_to_optimize_ * all_key_frames_.at(all_index).pose;
      all_index++;
    }
    return true;    
  }

  bool Viewer::JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    JointCloudMap(optimized_key_frames_, global_map_ptr);
    return true;
  }

  bool Viewer::JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    size_t begin_index = 0;
    if(all_key_frames_.size() > (size_t)local_frame_num_)//关键帧总数大于滑窗大小
      begin_index = all_key_frames_.size() - (size_t)local_frame_num_;//滑窗起始关键帧的序号
    std::deque<KeyFrame> local_key_frames;
    for(size_t i = begin_index; i < all_key_frames_.size(); ++i) {
      local_key_frames.push_back(all_key_frames_.at(i));
    }
    JointCloudMap(local_key_frames,local_map_ptr);
    return true;
  }

  //地图中的每一帧点云=该帧点云*该帧的位姿
  //因为此时每一帧都已经优化(参见函数OptimizeKeyFrames)，所以得到的是按优化后的位姿投影之后的点云
  bool Viewer::JointCloudMap(const std::deque<KeyFrame>& key_frames, CloudData::CLOUD_PTR& map_cloud_ptr) {
    map_cloud_ptr.reset(new CloudData::CLOUD());
    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    std::string file_path = "";
    for(size_t i=0; i < key_frames.size(); ++i) {
      file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
      pcl::io::loadPCDFile(file_path, *cloud_ptr);
      pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
      *map_cloud_ptr += *cloud_ptr;
    }
    return true;
  }

  bool Viewer::SaveMap() {
    if(optimized_key_frames_.size() == 0)
      return false;
    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    JointCloudMap(optimized_key_frames_, global_map_ptr);

    std::string map_file_path = map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);

    LOG(INFO) << "地图保存完成，地址是：" << std::endl << map_file_path << std::endl << std::endl;

    return true;
  }

  Eigen::Matrix4f& Viewer::GetCurrentPose() {
    return optimized_odom_.pose;
  }

  CloudData::CLOUD_PTR& Viewer::GetCurrentScan() {
    frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr, optimized_cloud_.cloud_ptr);
    return optimized_cloud_.cloud_ptr;
  }

  bool Viewer::GetLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    JointLocalMap(local_map_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr, local_map_ptr);
    return true;
  }

  bool Viewer::GetGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    JointGlobalMap(global_map_ptr);
    global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
    return true;
  }

  bool Viewer::HasNewLocalMap() {
    return has_new_local_map_;
  }

  bool Viewer::HasNewGlobalMap() {
    return has_new_global_map_;
  }
}