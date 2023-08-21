/*
Description:
Author     : Wang Junpeng
date       :
*/

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

struct LidarEdgeFactor
{
  LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_, double s_)
      : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  { // 旋转、平移、残差，重载了（）括号运算符
    // 将double数组转换成eigen的数据结构，注意这里必须都写成模板
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
    Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

    // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
    Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]}; // 上一时刻到这一时刻的变换
    Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};

    // 计算的是上一帧到当前帧的位姿变换，因此根据匀速模型，计算该点对应的位姿
    // 这里暂时不考虑畸变，因此这里不做任何变换
    // slerp线性插值函数；Quaternion.Slerp(a,b,t)，作用：将物体从a以t的选择速度转向b。
    q_last_curr = q_identity.slerp(T(s), q_last_curr);
    Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]}; // 平移的插值

    Eigen::Matrix<T, 3, 1> lp; // 在上一帧的坐标系中的坐标
    // 把当前点根据计算的帧间位姿变换到上一帧的坐标系中
    lp = q_last_curr * cp + t_last_curr;

    Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb); // 模是三角形的面积；cross是向量的叉乘，结果为向量
    Eigen::Matrix<T, 3, 1> de = lpa - lpb;                  // 底边的向量
    // 残差为向量；残差的模是该点到底边的垂线长度，即该点到对应的面/线特征的距离
    residual[0] = nu.x() / de.norm(); // 计算残差时用的是其平方和，因此不用关心正负
    residual[1] = nu.y() / de.norm();
    residual[2] = nu.z() / de.norm();

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                     const Eigen::Vector3d last_point_b_, const double s_)
  { //(重载operate运算符的类, 残差的维数, 参数块的维数（四元数，位置）)
    return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(
        new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));

    // new Lidar EdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)：新建实例
    // ceres::AutoDiffCostFunction<Lidar EdgeFactor, 3, 4, 3>(new Lidar EdgeFactor)
  }

  Eigen::Vector3d curr_point, last_point_a, last_point_b;
  double s;
};

struct LidarPlaneFactor
{
  LidarPlaneFactor(Eigen::Vector3d curr_point_,
                   Eigen::Vector3d last_point_j_,
                   Eigen::Vector3d last_point_l_,
                   Eigen::Vector3d last_point_m_,
                   double s_)
      : curr_point(curr_point_), // 作为构造函数的传入
        last_point_j(last_point_j_),
        last_point_l(last_point_l_),
        last_point_m(last_point_m_),
        s(s_)
  {
    // 平面的单位法向量
    // 向量叉乘结果垂直于原来的两个向量
    ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
    // 变为单位向量
    ljm_norm.normalize();
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
    // Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
    // Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
    Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())}; // 法向量

    // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
    Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
    Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
    // 根据时间插值
    q_last_curr = q_identity.slerp(T(s), q_last_curr);
    Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = q_last_curr * cp + t_last_curr; // 当前点转换到上一帧的坐标系中

    residual[0] = (lp - lpj).dot(ljm); // 向量的点乘，点到平面的距离，为标量

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
                                     const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
                                     const double s_)
  {
    return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
        new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
  }

  Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
  Eigen::Vector3d ljm_norm;
  double s;
};

struct LidarPlaneNormFactor
{
  LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_)
      : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr; // 投影到地图坐标系

    Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
    // 已经归一化，模为1,因此下式省略了“除以向量的模”；ceres的原理为最小化残差的平方，因此下式不需要取模
    residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm); // 求解点到平面的距离
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
                                     const double negative_OA_dot_norm_)
  {
    return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactor, 1, 4, 3>(
        new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d plane_unit_norm;
  double negative_OA_dot_norm;
};

struct LidarDistanceFactor
{
  LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_)
      : curr_point(curr_point_), closed_point(closed_point_) {}

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const
  {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;

    residual[0] = point_w.x() - T(closed_point.x());
    residual[1] = point_w.y() - T(closed_point.y());
    residual[2] = point_w.z() - T(closed_point.z());
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
  {
    return (new ceres::AutoDiffCostFunction<LidarDistanceFactor, 3, 4, 3>(
        new LidarDistanceFactor(curr_point_, closed_point_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d closed_point;
};