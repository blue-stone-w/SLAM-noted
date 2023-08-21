/*
Description: tf监听模块
*/

#include "frame/tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>

namespace frame {
  TFListener::TFListener(ros::NodeHandle& nh,
                         std::string base_frame_id,
                         std::string child_frame_id)
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
    }

  bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try {
      //tf::StampedTransform在tf.h中出现，没找到该变量的定义
      tf::StampedTransform transform;

      //lookupTransform在tf.h中的class Transformer
      //listener_的类为transform_listener.h中的class TransformListener
      // 返回两个坐标系的变换,返回的变换的方向将从base_frame到child_frame
      listener_.lookupTransform(base_frame_id_,child_frame_id_, ros::Time(0), transform);
      TransformToMatrix(transform, transform_matrix);
      return true;
    } catch (tf::TransformException &ex) {
        return false;
    }
  }

  bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    //此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol *rot_x_btol).matrix();

    return true;
  }
}