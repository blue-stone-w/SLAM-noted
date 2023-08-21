/*
Description: NDT匹配模块
Author     : Wang Junpeng
data       : 
*/

#include "frame/models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace frame {
  NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
      float res = node["res"].as<float>();
      float step_size = node["step_size"].as<float>();
      float trans_eps = node["trans_eps"].as<float>();
      int max_iter = node["max_iter"].as<int>();

      SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

  NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
   :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
     SetRegistrationParam(res, step_size, trans_eps, max_iter);
   }

  bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    //设置ndt的匹配参数
    ndt_ptr_->setResolution(res);//分辨率
    ndt_ptr_->setStepSize(step_size);//步长
    ndt_ptr_->setTransformationEpsilon(trans_eps);//转换误差
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT的匹配参数" << std::endl
              << "res: " << res << ", "
              << "step_size: "  << step_size << ", "
              << "trans_eps: "  << trans_eps << ", "
              << "max_iter: " << max_iter
              << std::endl << std::endl;
    return true;
  }

  bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);//小地图/滑窗

    return true;
  }

  bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                  const Eigen::Matrix4f& predict_pose,
                                  CloudData::CLOUD_PTR& result_cloud_ptr,
                                  Eigen::Matrix4f& result_pose) {
    ndt_ptr_->setInputSource(input_source);//滤波后的点云
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);//匹配
    result_pose = ndt_ptr_->getFinalTransformation();//得到位姿

    return true;
  }

  float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
  }
}