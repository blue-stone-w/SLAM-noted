/*
Description: NDT匹配模块
Author     : Wang Junpeng
data       : 
*/

#ifndef FRAME_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define FRAME_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "frame/models/registration/registration_interface.hpp"

namespace frame {
  class NDTRegistration: public RegistrationInterface {
    public:
      NDTRegistration(const YAML::Node& node);
      NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

      bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
      bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                     const Eigen::Matrix4f& predit_pose,
                     CloudData::CLOUD_PTR& result_cloud_ptr,
                     Eigen::Matrix4f& result_pose) override;
      float GetFitnessScore() override;

    private:
      bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    private:
      pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
  };
}
#endif