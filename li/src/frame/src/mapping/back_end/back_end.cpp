/*
Description: back end具体实现
Author     : Wang Junpeng
date       : 
*/

#include "frame/mapping/back_end/back_end.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "frame/global_defination/global_defination.h"
#include "frame/tools/file_manager.hpp"

namespace frame {
  BackEnd::BackEnd() {
    InitWithConfig();
  }

  bool BackEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/back_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------后端初始化-------------------" << std::endl;
    InitParam(config_node);
    InitGraphOptimizer(config_node);
    InitDataPath(config_node);

    return true;
  }

  bool BackEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    
    return true;
  }

  bool BackEnd::InitGraphOptimizer(const YAML::Node& config_node) {
    std::string graph_optimizer_type = config_node["graph_optimizer_type"].as<std::string>();
    if(graph_optimizer_type == "g2o") {
      //使用LM迭代
      graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>("lm_var");
    } else {
      LOG(ERROR) << "没有找到与 " << graph_optimizer_type << " 对应的图优化模式,请检查配置文件";
      return false;
    }
    //后续可用if-else语句自行添加其它的优化器,例如gtsam
    std::cout << "后端优化选择的优化器为：" << graph_optimizer_type << std::endl << std::endl;

    graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = config_node["use_loop_close"].as<bool>();
    
    graph_optimizer_config_.optimize_step_with_key_frame = config_node["optimize_step_with_key_frame"].as<int>();
    graph_optimizer_config_.optimize_step_with_gnss = config_node["optimize_step_with_gnss"].as<int>();
    graph_optimizer_config_.optimize_step_with_loop = config_node["optimize_step_with_loop"].as<int>();
    
    for(int i=0; i<6; ++i) {
      graph_optimizer_config_.odom_edge_noise(i) = config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i].as<double>();
      graph_optimizer_config_.close_loop_noise(i) = config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i].as<double>();
     }
     
     for (int i = 0; i < 3; i++) {
        graph_optimizer_config_.gnss_noise(i) = config_node[graph_optimizer_type + "_param"]["gnss_noise"][i].as<double>();
    }

    return true;
  }

  bool BackEnd::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if(data_path == "./") {
      data_path = WORK_SPACE_PATH;
    }
    
    if(!FileManager::CreateDirectory(data_path + "/slam_data"))
      return false;
    key_frames_path_ = data_path + "/slam_data/key_frames";
    std::cout << key_frames_path_ << std::endl << std::endl;
    trajectory_path_ = data_path + "/slam_data/trajectory";
    std::cout << trajectory_path_ << std::endl << std::endl;

    if(!FileManager::InitDirectory(key_frames_path_, "关键帧点云"))
      return false;
    std::cout << "3datapath start" << std::endl << std::endl;

    if(!FileManager::InitDirectory(trajectory_path_, "轨迹文件"))
      return false;
    
    if(!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
      return false;
    std::cout << "4datapath start" << std::endl << std::endl;
    if(!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
    return false;
    std::cout << "datapath done" << std::endl << std::endl;

    return true;
  }

  bool BackEnd::Update(const CloudData& cloud_data, 
                       const PoseData& laser_odom, 
                       const PoseData& gnss_pose) {
    ResetParam();//默认没有新的关键帧和新的已优化的关键帧

    if(MaybeNewKeyFrame(cloud_data, laser_odom, gnss_pose)) {//根据距离判断是否为关键帧并存储关键帧为pcd格式
      SavePose(ground_truth_ofs_, gnss_pose.pose);//存储位姿为txt文件
      SavePose(laser_odom_ofs_, laser_odom.pose);
      AddNodeAndEdge(gnss_pose);
      if(MaybeOptimized()) {//根据关键帧、gnss和闭环的数量决定是否需要更新，若需要则执行更新；返回是否已更新
        SaveOptimizedPose();
      }
    }

    return true;
  }

  bool BackEnd::InsertLoopPose(const LoopPose& loop_pose) {
    if(!graph_optimizer_config_.use_loop_close)//判断是否使用闭环检测
      return false;

    Eigen::Isometry3d isometry;//是维持任意两点距离不变的仿射变换，也称做欧氏变换、刚体运动
    isometry.matrix() = loop_pose.pose.cast<double>();
    //参数为（节点1,节点2,变换矩阵/相对位姿，噪声）
    graph_optimizer_ptr_->AddSe3Edge(loop_pose.index0, loop_pose.index1, isometry, graph_optimizer_config_.close_loop_noise);
    new_loop_cnt_ ++;
    LOG(INFO) << "插入闭环：" << loop_pose.index0 << "," << loop_pose.index1;

    return true;
  }

  void BackEnd::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
  }

  bool BackEnd::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        ofs << pose(i, j);
        if(i==2 && j==3) {
          ofs << std::endl;//该pose数据已经完全输入ofs
        } else {
          ofs << " ";
        }
      }
    }
    return true;
  }

  bool BackEnd::MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_odom) {
    static Eigen::Matrix4f last_key_pose = laser_odom.pose;
    if(key_frames_deque_.size() == 0) {
      has_new_key_frame_ = true;
      last_key_pose = laser_odom.pose;
    }

    //匹配后根据距离判断是否生成新的关键帧，若生成则做相应更新
    if(fabs(laser_odom.pose(0,3) - last_key_pose(0,3)) + 
       fabs(laser_odom.pose(1,3) - last_key_pose(1,3)) +
       fabs(laser_odom.pose(2,3) - last_key_pose(2,3)) > key_frame_distance_) {
       
       has_new_key_frame_ = true;
       last_key_pose = laser_odom.pose;
    }

    if(has_new_key_frame_) {
      //把关键帧的点云存储进硬盘
      std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
      pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);
      
      KeyFrame key_frame;
      key_frame.time = laser_odom.time;
      key_frame.index = (unsigned int)key_frames_deque_.size();
      key_frame.pose = laser_odom.pose;
      key_frames_deque_.push_back(key_frame);
      current_key_frame_ = key_frame;

      current_key_gnss_.time = gnss_odom.time;
      current_key_gnss_.index = key_frame.index;
      current_key_gnss_.pose = gnss_odom.pose;
    }

    return has_new_key_frame_;
  }

  bool BackEnd::AddNodeAndEdge(const PoseData& gnss_data) {
    Eigen::Isometry3d isometry;
    //添加关键帧节点
    isometry.matrix() = current_key_frame_.pose.cast<double>();
    if(!graph_optimizer_config_.use_gnss && graph_optimizer_ptr_->GetNodeNum() == 0) {//不使用gnss，且没有节点
      graph_optimizer_ptr_->AddSe3Node(isometry, true);//从g2o_graph_optimizer.cpp调用的函数
    } else {
      graph_optimizer_ptr_->AddSe3Node(isometry, false);
    }
    new_key_frame_cnt_++;

    //添加激光里程计对应的边
    static KeyFrame last_key_frame = current_key_frame_;
    int node_num = graph_optimizer_ptr_->GetNodeNum();
    if(node_num>1) { //至少有两个节点才能添加边
      Eigen::Matrix4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;
      isometry.matrix() = relative_pose.cast<double>();
      graph_optimizer_ptr_->AddSe3Edge(node_num-2, node_num-1, isometry, graph_optimizer_config_.odom_edge_noise);
    }
    last_key_frame = current_key_frame_;

    //添加gnss对应的先验边
    if(graph_optimizer_config_.use_gnss) {
      Eigen::Vector3d xyz(static_cast<double>(gnss_data.pose(0,3)),
                          static_cast<double>(gnss_data.pose(1,3)),
                          static_cast<double>(gnss_data.pose(2,3)));
      graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num-1, xyz, graph_optimizer_config_.gnss_noise);
      new_gnss_cnt_++;
    }
    return true;
  }

  bool BackEnd::MaybeOptimized() {
    bool need_optimize = false;
    //全局点云地图本身比较大，产生和发送地图都非常耗时间，所以我们目前把优化控制在一个很低的频率上。
    //当三个计数之一达到配置中设置的值时，即做优化，并把三个计数均清零。
    if(new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss)
      need_optimize = true;
    if(new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop)
      need_optimize = true;
    if(new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame)
       need_optimize = true;

    if(!need_optimize)
      return false;

    new_gnss_cnt_ = 0;
    new_loop_cnt_ = 0;
    new_key_frame_cnt_ = 0;

    if(graph_optimizer_ptr_->Optimize())
      has_new_optimized_ = true;

    return true;
  }

  bool BackEnd::SaveOptimizedPose() {
    if(graph_optimizer_ptr_->GetNodeNum() == 0)
      return false;
    if(!FileManager::CreateFile(optimized_pose_ofs_, trajectory_path_ + "/optimized.txt"))
      return false;
    graph_optimizer_ptr_->GetOptimizedPose(optimized_pose_);
    for(size_t i=0; i<optimized_pose_.size(); ++i) {
      SavePose(optimized_pose_ofs_, optimized_pose_.at(i));
    }

    return true;
  }

  bool BackEnd::ForceOptimize() {
    if(graph_optimizer_ptr_->Optimize())
      has_new_optimized_ = true;
    SaveOptimizedPose();
    return has_new_optimized_;
  }

  void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frame_deque) {
    KeyFrame key_frame;
    for(size_t i=0; i<optimized_pose_.size(); ++i) {
      key_frame.pose = optimized_pose_.at(i);
      key_frame.index = (unsigned int)i;
      key_frame_deque.push_back(key_frame);
    }
  }

  bool BackEnd::HasNewKeyFrame() {
    return has_new_key_frame_;
  }

  bool BackEnd::HasNewOptimized() {
    return has_new_optimized_;
  }

  void BackEnd::GetLatestKeyFrame(KeyFrame& key_frame) {
    key_frame = current_key_frame_;
  }

  void BackEnd::GetLatestKeyGNSS(KeyFrame& key_frame) {
    key_frame = current_key_gnss_;
  }
} 