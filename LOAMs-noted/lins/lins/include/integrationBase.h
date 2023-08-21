/*
---Description---
between two key frames
integration: Xk -> Xk+1
*/

#ifndef INCLUDE_INTEGRATIONBASE_H_
#define INCLUDE_INTEGRATIONBASE_H_

#include <math_utils.h>
#include <parameters.h>
#include <KalmanFilter.hpp>

#include <cassert>
#include <cmath>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace filter;

namespace integration
{
enum StateOrder
{
  O_R  = 0,
  O_P  = 3,
  O_V  = 6,
  O_BA = 9,
  O_BG = 12
}; // unused
enum NoiseOrder
{
  O_AN = 0,
  O_GN = 3,
  O_AW = 6,
  O_GW = 9
}; // unused

const double ACC_N = 1e-4; // noise
const double GYR_N = 1e-4;
const double ACC_W = 1e-8; // 状态转移方程的数学模型忽略了重力，因此产生的模型噪声
const double GYR_W = 1e-8;

class IntegrationBase
{
 public:
  Eigen::Vector3d G;

  double dt;
  Eigen::Vector3d acc_0, gyr_0; // old data
  Eigen::Vector3d acc_1, gyr_1; // new data

  const Eigen::Vector3d linearized_acc, linearized_gyr;
  Eigen::Vector3d linearized_ba, linearized_bg;

  Eigen::Matrix<double, 15, 15> jacobian, covariance;
  Eigen::Matrix<double, 15, 15> step_jacobian; // 没有使用，可能原来是单步积分时的雅可比
  Eigen::Matrix<double, 15, 18> step_V; // 没有使用，可能是单步积分时的状态
  Eigen::Matrix<double, 18, 18> noise;

  double sum_dt; // since last key frame
  Eigen::Vector3d delta_p; // 位移增量since last key frame
  Eigen::Quaterniond delta_q; // 旋转增量
  Eigen::Vector3d delta_v; // 速度增量

  std::vector<double> dt_buf; // save measurements
  std::vector<Eigen::Vector3d> acc_buf;
  std::vector<Eigen::Vector3d> gyr_buf;

  IntegrationBase( ) = delete; // delete default constructor
  IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                  const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg) :
    acc_0{_acc_0}, // initialize members in this class; _acc_0 -> acc_0
    gyr_0{_gyr_0}, // _gyr_0 to data member gyr_0
    linearized_acc{_acc_0},
    linearized_gyr{_gyr_0},
    linearized_ba{_linearized_ba},
    linearized_bg{_linearized_bg},
    jacobian{Eigen::Matrix<double, 15, 15>::Identity( )},
    covariance{Eigen::Matrix<double, 15, 15>::Zero( )},
    sum_dt{0.0},
    delta_p{Eigen::Vector3d::Zero( )},
    delta_q{Eigen::Quaterniond::Identity( )},
    delta_v{Eigen::Vector3d::Zero( )}
  {}

  void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
  {
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr); // integrate: last state + (old vel + new vel)/2*dt
  }

  void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
  {
    dt    = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    // 中值积分: 更新雅可比矩阵; 返回值是新的delta，不会改变bias; 返回的delta和输入的delta在同一坐标系下
    midPointIntegration(_dt, acc_0, gyr_0, acc_1, _gyr_1, // in(old and new measurement)
                        delta_p, delta_q, delta_v, // in(old value)
                        linearized_ba, linearized_bg, // in
                        result_delta_p, result_delta_q, result_delta_v, // out
                        result_linearized_ba, result_linearized_bg, 1); // out; point at mid time

    // checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v, linearized_ba, linearized_bg);
    delta_p       = result_delta_p;
    delta_q       = result_delta_q;
    delta_v       = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize( );
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
  }

  // 中值 积分
  void midPointIntegration(
      double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
      const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
      const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
      const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
      Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
      Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
  {
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg; // 角速度的中值
    result_delta_q  = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2); // 假设theta很小所做的近似

    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba); // 上一时刻的加速度转换到上一关键帧
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Vector3d un_acc   = 0.5 * (un_acc_0 + un_acc_1); // 在参考系下加速度的均值，不含bias，包含了重力

    result_delta_p       = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt; // 相对于上一关键帧
    result_delta_v       = delta_v + un_acc * _dt; // 都是相对于上一estimator时刻的
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian)
    { // 对雅可比矩阵进行迭代
      Vector3d w_x   = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg; // angular_velocity
      Vector3d a_0_x = _acc_0 - linearized_ba; // acceleration measurement - bias
      Vector3d a_1_x = _acc_1 - linearized_ba;
      Matrix3d R_w_x, R_a_0_x, R_a_1_x; // 对应论文公式(6)

      // 反对称阵
      R_w_x << 0, -w_x(2), w_x(1), // [w-b]x, cross product
          w_x(2), 0, -w_x(0),
          -w_x(1), w_x(0), 0;

      R_a_0_x << 0, -a_0_x(2), a_0_x(1), // [w-a]x
          a_0_x(2), 0, -a_0_x(0),
          -a_0_x(1), a_0_x(0), 0;

      R_a_1_x << 0, -a_1_x(2), a_1_x(1),
          a_1_x(2), 0, -a_1_x(0),
          -a_1_x(1), a_1_x(0), 0;

      // 原论文公式有误，代码中修正。the order of a and theta is exchanged. and F = I + F*dt + 0.5*F^2*dt^2
      MatrixXd F = MatrixXd::Zero(15, 15);

      // pos in F
      F.block<3, 3>(GlobalState::pos_, GlobalState::pos_) = Matrix3d::Identity( );
      // last rot * last acc; curr rot * curr acc * (I-drot)=currot*curacc-currot*acc*drot;
      // 0.25=0.5*0.5 ---> (0.5a*t^2); 0.5*last+0.5curr
      F.block<3, 3>(GlobalState::pos_, GlobalState::att_) =
          -0.25 * delta_q.toRotationMatrix( ) * R_a_0_x * _dt * _dt
          + -0.25 * result_delta_q.toRotationMatrix( ) * R_a_1_x * (Matrix3d::Identity( ) - R_w_x * _dt) * _dt * _dt;

      F.block<3, 3>(GlobalState::pos_, GlobalState::vel_) = MatrixXd::Identity(3, 3) * _dt;
      // 0.5*(last+curr) * 0.5*t*t
      F.block<3, 3>(GlobalState::pos_, GlobalState::acc_) =
          -0.25 * (delta_q.toRotationMatrix( ) + result_delta_q.toRotationMatrix( )) * _dt * _dt;
      // rot*acc
      F.block<3, 3>(GlobalState::pos_, GlobalState::gyr_) =
          -0.25 * result_delta_q.toRotationMatrix( ) * R_a_1_x * _dt * _dt * -_dt;

      // att in F
      F.block<3, 3>(GlobalState::att_, GlobalState::att_) = Matrix3d::Identity( ) - R_w_x * _dt;
      F.block<3, 3>(GlobalState::att_, GlobalState::gyr_) = -1.0 * MatrixXd::Identity(3, 3) * _dt;

      // vel in F
      F.block<3, 3>(GlobalState::vel_, GlobalState::att_) =
          -0.5 * delta_q.toRotationMatrix( ) * R_a_0_x * _dt + -0.5 * result_delta_q.toRotationMatrix( ) * R_a_1_x * (Matrix3d::Identity( ) - R_w_x * _dt) * _dt;
      F.block<3, 3>(GlobalState::vel_, GlobalState::vel_) = Matrix3d::Identity( );
      F.block<3, 3>(GlobalState::vel_, GlobalState::acc_) =
          -0.5 * (delta_q.toRotationMatrix( ) + result_delta_q.toRotationMatrix( )) * _dt;
      F.block<3, 3>(GlobalState::vel_, GlobalState::gyr_) =
          -0.5 * result_delta_q.toRotationMatrix( ) * R_a_1_x * _dt * -_dt;

      F.block<3, 3>(GlobalState::acc_, GlobalState::acc_) = Matrix3d::Identity( );
      F.block<3, 3>(GlobalState::gyr_, GlobalState::gyr_) = Matrix3d::Identity( );

      jacobian = F * jacobian;
    }

    /*
    Vector3d vk0 = _acc_0*dt;
    Vector3d ak0 = _gyr_0*dt;
    Vector3d vk1 = _acc_1*dt;
    Vector3d ak1 = _gyr_1*dt;

    Vector3d dv = vk1 + 0.5*ak1.cross(vk1) + 1.0/12*(ak0.cross(vk1) +
    vk0.cross(ak1)); Vector3d da = 0.5*(ak0+ak1);

    result_delta_q = delta_q * Quaterniond(1, da(0)/2, da(1)/2, da(2)/2);
    result_delta_v = delta_v + result_delta_q*dv;
    Vector3d aver_v = 0.5*(result_delta_v + delta_v);
    result_delta_p = delta_p + aver_v * _dt;

    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;*/
  }

  void setBa(const Eigen::Vector3d &ba) { linearized_ba = ba; }

  void setBg(const Eigen::Vector3d &bg) { linearized_bg = bg; }
};


} // namespace integration

#endif // INCLUDE_INTEGRATIONBASE_H_