/* done
#Description: this file contains two classes: GlobalState & StatePredictor
GlobalState - variables and box operator
StatePredictor - predict state and covariance, set & reset, update, initialization, initialize covariance
*/

#ifndef INCLUDE_KALMANFILTER_HPP_
#define INCLUDE_KALMANFILTER_HPP_

#include <math_utils.h>
#include <parameters.h>

#include <iostream>
#include <map>

using namespace std;
using namespace math_utils;
using namespace parameter;

namespace filter
{

// GlobalState Class contains state variables including position, velocity,
// attitude, acceleration bias, gyroscope bias, and gravity
class GlobalState
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // variable's position
  static constexpr unsigned int DIM_OF_STATE_ = 18;
  static constexpr unsigned int DIM_OF_NOISE_ = 12;
  static constexpr unsigned int pos_          = 0;
  static constexpr unsigned int vel_          = 3;
  static constexpr unsigned int att_          = 6; // attitude angle: roll, pitch, yaw
  static constexpr unsigned int acc_          = 9;
  static constexpr unsigned int gyr_          = 12; // bw_, gyroscope bias
  static constexpr unsigned int gra_          = 15; // gn, gravity

  // State
  V3D rn_; // position in n-frame(coordinate)
  V3D vn_; // velocity in n-frame
  Q4D qbn_; // rotation from b-frame to n-frame
  V3D ba_; // acceleartion bias
  V3D bw_; // gyroscope bias
  V3D gn_; // gravity

  GlobalState( ) { setIdentity( ); }

  void setIdentity( )
  {
    rn_.setZero( );
    vn_.setZero( );
    qbn_.setIdentity( );
    ba_.setZero( );
    bw_.setZero( );
    gn_ << 0.0, 0.0, -G0;
  }

  GlobalState(const V3D &rn, const V3D &vn, const Q4D &qbn, const V3D &ba, const V3D &bw)
  {
    setIdentity( ); // initialize glabal state
    rn_  = rn;
    vn_  = vn;
    qbn_ = qbn;
    ba_  = ba;
    bw_  = bw;
  }

  ~GlobalState( ) {}

  // boxPlus operator to get setimated state
  void boxPlus(const Eigen::Matrix<double, DIM_OF_STATE_, 1> &xk, // estimated error-stated
               GlobalState &stateOut)
  { // estimated state
    stateOut.rn_  = rn_ + xk.template segment<3>(pos_); // read 3 data from index(pos_)
    stateOut.vn_  = vn_ + xk.template segment<3>(vel_); // global state + error-state
    stateOut.ba_  = ba_ + xk.template segment<3>(acc_);
    stateOut.bw_  = bw_ + xk.template segment<3>(gyr_);
    Q4D dq        = axis2Quat(xk.template segment<3>(att_));
    stateOut.qbn_ = (qbn_ * dq).normalized( );
    stateOut.gn_  = gn_ + xk.template segment<3>(gra_);
  }

  // boxMinus operator to get estimated error-state
  void boxMinus(const GlobalState &stateIn, // updated state
                Eigen::Matrix<double, DIM_OF_STATE_, 1> &xk)
  { // estimated error-state
    xk.template segment<3>(pos_) = rn_ - stateIn.rn_; // global state - error-state
    xk.template segment<3>(vel_) = vn_ - stateIn.vn_;
    xk.template segment<3>(acc_) = ba_ - stateIn.ba_;
    xk.template segment<3>(gyr_) = bw_ - stateIn.bw_;
    V3D da                       = Quat2axis(stateIn.qbn_.inverse( ) * qbn_);
    xk.template segment<3>(att_) = da;
    xk.template segment<3>(gra_) = gn_ - stateIn.gn_;
  }

  GlobalState &operator=(const GlobalState &other)
  { // then we can use "=" to assignment
    if (this == &other)
      return *this;

    this->rn_  = other.rn_;
    this->vn_  = other.vn_;
    this->qbn_ = other.qbn_;
    this->ba_  = other.ba_;
    this->bw_  = other.bw_;
    this->gn_  = other.gn_;

    return *this;
  }
};

class StatePredictor
{
 public:
  // Eigen库为了使用SSE加速，所以内存分配上使用了128位的指针;宏:数据结构内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW GlobalState state_; // 绝对是两关键帧之间的相对位姿
  double time_;

  Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_> F_;
  Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_> jacobian_, covariance_;
  Eigen::Matrix<double, GlobalState::DIM_OF_NOISE_, GlobalState::DIM_OF_NOISE_> noise_; // 模型的噪声

  V3D acc_last; // last acceleration measurement
  V3D gyr_last; // last gyroscope measurement

  bool flag_init_state_;
  bool flag_init_imu_;

  StatePredictor( ) { reset( ); }

  void reset(int type = 0)
  {
    if (type == 0)
    { // Initialize using offline parameters
      state_.rn_.setZero( );
      state_.vn_ = state_.qbn_.inverse( ) * state_.vn_;
      state_.qbn_.setIdentity( );
      initializeCovariance( );
    }
    else if (type == 1)
    { // Inheritage previous covariance and state
      V3D covPos      = INIT_POS_STD.array( ).square( );
      double covRoll  = pow(deg2rad(INIT_ATT_STD(0)), 2);
      double covPitch = pow(deg2rad(INIT_ATT_STD(1)), 2);
      double covYaw   = pow(deg2rad(INIT_ATT_STD(2)), 2);

      M3D vel_cov = covariance_.block<3, 3>(GlobalState::vel_, GlobalState::vel_);
      M3D acc_cov = covariance_.block<3, 3>(GlobalState::acc_, GlobalState::acc_);
      M3D gyr_cov = covariance_.block<3, 3>(GlobalState::gyr_, GlobalState::gyr_);
      M3D gra_cov = covariance_.block<3, 3>(GlobalState::gra_, GlobalState::gra_);

      covariance_.setZero( );
      covariance_.block<3, 3>(GlobalState::pos_, GlobalState::pos_) = covPos.asDiagonal( ); // pos
      covariance_.block<3, 3>(GlobalState::vel_, GlobalState::vel_) = state_.qbn_.inverse( ) * vel_cov * state_.qbn_; // vel
      covariance_.block<3, 3>(GlobalState::att_, GlobalState::att_) = V3D(covRoll, covPitch, covYaw).asDiagonal( ); // att
      covariance_.block<3, 3>(GlobalState::acc_, GlobalState::acc_) = acc_cov;
      covariance_.block<3, 3>(GlobalState::gyr_, GlobalState::gyr_) = gyr_cov;
      covariance_.block<3, 3>(GlobalState::gra_, GlobalState::gra_) = state_.qbn_.inverse( ) * gra_cov * state_.qbn_; // 重力协方差传递

      state_.rn_.setZero( );
      state_.vn_ = state_.qbn_.inverse( ) * state_.vn_;
      state_.qbn_.setIdentity( );
      state_.gn_ = state_.qbn_.inverse( ) * state_.gn_;
      state_.gn_ = state_.gn_ * 9.81 / state_.gn_.norm( ); // 对重力做归一化，保证大小为9.81
      // 以上，对相对位置和姿态归零，速度和重力保留但是转换到当前时刻姿态下，bias不变
      // 相应的，位置和姿态的协方差初始化，速度和重力的协方差传递，bias方差不变
      // initializeCovariance(1);
    }
  }
  void reset(V3D vn, V3D ba, V3D bw)
  {
    state_.setIdentity( );
    state_.vn_ = vn;
    state_.ba_ = ba;
    state_.bw_ = bw;
    initializeCovariance( );
  }

  void initializeCovariance(int type = 0)
  {
    double covX     = pow(INIT_POS_STD(0), 2); // INIT_POS_STD is in parameters.cpp
    double covY     = pow(INIT_POS_STD(1), 2);
    double covZ     = pow(INIT_POS_STD(2), 2);
    double covVx    = pow(INIT_VEL_STD(0), 2);
    double covVy    = pow(INIT_VEL_STD(1), 2);
    double covVz    = pow(INIT_VEL_STD(2), 2);
    double covRoll  = pow(deg2rad(INIT_ATT_STD(0)), 2);
    double covPitch = pow(deg2rad(INIT_ATT_STD(1)), 2);
    double covYaw   = pow(deg2rad(INIT_ATT_STD(2)), 2);

    V3D covPos = INIT_POS_STD.array( ).square( ); // square(every element)/每个元素求平方
    V3D covVel = INIT_VEL_STD.array( ).square( );
    V3D covAcc = INIT_ACC_STD.array( ).square( );
    V3D covGyr = INIT_GYR_STD.array( ).square( );

    double peba  = pow(ACC_N * ug, 2); // ug:micro-gravity force
    double pebg  = pow(GYR_N * dph, 2);
    double pweba = pow(ACC_W * ugpsHz, 2); // 忽略重力产生的噪声
    double pwebg = pow(GYR_W * dpsh, 2);
    V3D gra_cov(0.01, 0.01, 0.01);

    if (type == 0)
    { // Initialize using offline parameters
      covariance_.setZero( );
      // elements of vector are diagonal elements of matrix
      covariance_.block<3, 3>(GlobalState::pos_, GlobalState::pos_) = covPos.asDiagonal( ); // pos
      covariance_.block<3, 3>(GlobalState::vel_, GlobalState::vel_) = covVel.asDiagonal( ); // vel
      covariance_.block<3, 3>(GlobalState::att_, GlobalState::att_) = V3D(covRoll, covPitch, covYaw).asDiagonal( ); // att
      covariance_.block<3, 3>(GlobalState::acc_, GlobalState::acc_) = covAcc.asDiagonal( ); // ba
      covariance_.block<3, 3>(GlobalState::gyr_, GlobalState::gyr_) = covGyr.asDiagonal( ); // bg
      covariance_.block<3, 3>(GlobalState::gra_, GlobalState::gra_) = gra_cov.asDiagonal( ); // gravity
    }
    else if (type == 1)
    { // Inheritage previous covariance
      M3D vel_cov = covariance_.block<3, 3>(GlobalState::vel_, GlobalState::vel_);
      M3D acc_cov = covariance_.block<3, 3>(GlobalState::acc_, GlobalState::acc_);
      M3D gyr_cov = covariance_.block<3, 3>(GlobalState::gyr_, GlobalState::gyr_);
      M3D gra_cov = covariance_.block<3, 3>(GlobalState::gra_, GlobalState::gra_);

      covariance_.setZero( );
      covariance_.block<3, 3>(GlobalState::pos_, GlobalState::pos_) = covPos.asDiagonal( ); // pos
      covariance_.block<3, 3>(GlobalState::vel_, GlobalState::vel_) = vel_cov; // vel
      covariance_.block<3, 3>(GlobalState::att_, GlobalState::att_) = V3D(covRoll, covPitch, covYaw).asDiagonal( ); // att
      covariance_.block<3, 3>(GlobalState::acc_, GlobalState::acc_) = acc_cov;
      covariance_.block<3, 3>(GlobalState::gyr_, GlobalState::gyr_) = gyr_cov;
      covariance_.block<3, 3>(GlobalState::gra_, GlobalState::gra_) = gra_cov;
    }

    noise_.setZero( );
    noise_.block<3, 3>(0, 0) = V3D(peba, peba, peba).asDiagonal( );
    noise_.block<3, 3>(3, 3) = V3D(pebg, pebg, pebg).asDiagonal( );
    noise_.block<3, 3>(6, 6) = V3D(pweba, pweba, pweba).asDiagonal( );
    noise_.block<3, 3>(9, 9) = V3D(pwebg, pwebg, pwebg).asDiagonal( );
  }

  // 根据IMU测量值预测
  bool predict(double dt, const V3D &acc, const V3D &gyr, bool update_jacobian_ = true)
  {
    if (!isInitialized( )) return false; // 是否已经有了初始状态，即初始化了一个相对位姿
    if (!flag_init_imu_)
    {
      flag_init_imu_ = true;
      acc_last       = acc;
      gyr_last       = gyr;
    }

    // Average acceleration and angular rate
    GlobalState state_tmp = state_;
    // 以下假设在相邻关键帧之间bias和重力g不变; state_tmp.gn_重力也是在上一关键帧坐标系下的
    V3D un_acc_0   = state_tmp.qbn_ * (acc_last - state_tmp.ba_) + state_tmp.gn_; // accelaration of last frame
    V3D un_gyr     = 0.5 * (gyr_last + gyr) - state_tmp.bw_; // Average angular rate
    Q4D dq         = axis2Quat(un_gyr * dt); // convert vector to quaternion
    state_tmp.qbn_ = (state_tmp.qbn_ * dq).normalized( ); // update quaternion
    V3D un_acc_1   = state_tmp.qbn_ * (acc - state_tmp.ba_) + state_tmp.gn_; // accelaration of current frame
    V3D un_acc     = 0.5 * (un_acc_0 + un_acc_1); // Average acceleration
    // 以上完成了对a和g的中值求解

    // State integral; 更新相对于上一关键帧的位置和姿态;
    state_tmp.rn_ = state_tmp.rn_ + dt * state_tmp.vn_ + 0.5 * dt * dt * un_acc;
    state_tmp.vn_ = state_tmp.vn_ + dt * un_acc;

    if (update_jacobian_)
    {
      // state transformation matrix; 对应论文公式(6)(论文公式不对)
      MXD Ft                                               = MXD::Zero(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_);
      Ft.block<3, 3>(GlobalState::pos_, GlobalState::vel_) = M3D::Identity( );
      // 此时state_tmp是新的状态
      Ft.block<3, 3>(GlobalState::vel_, GlobalState::att_) = -state_tmp.qbn_.toRotationMatrix( ) * skew(acc - state_tmp.ba_);
      Ft.block<3, 3>(GlobalState::vel_, GlobalState::acc_) = -state_tmp.qbn_.toRotationMatrix( );
      Ft.block<3, 3>(GlobalState::vel_, GlobalState::gra_) = M3D::Identity( ); // 这里应该差一个负号，论文也不对
      Ft.block<3, 3>(GlobalState::att_, GlobalState::att_) = -skew(gyr - state_tmp.bw_);
      Ft.block<3, 3>(GlobalState::att_, GlobalState::gyr_) = -M3D::Identity( );

      // noise transformation matrix; 对应论文公式(7)
      MXD Gt                               = MXD::Zero(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_NOISE_);
      Gt.block<3, 3>(GlobalState::vel_, 0) = -state_tmp.qbn_.toRotationMatrix( );
      Gt.block<3, 3>(GlobalState::att_, 3) = -M3D::Identity( );
      Gt.block<3, 3>(GlobalState::acc_, 6) = M3D::Identity( );
      Gt.block<3, 3>(GlobalState::gyr_, 9) = M3D::Identity( );
      Gt                                   = Gt * dt;

      const MXD I = MXD::Identity(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_);
      F_          = I + Ft * dt + 0.5 * Ft * Ft * dt * dt; // only one-order is saved in article, here second-order is saved

      // jacobian_ = F * jacobian_; 对应论文公式(11)
      covariance_ = F_ * covariance_ * F_.transpose( ) + Gt * noise_ * Gt.transpose( ); // covariance matrix P
      covariance_ = 0.5 * (covariance_ + covariance_.transpose( )).eval( ); // symmetric matrix
    }

    state_ = state_tmp;
    time_ += dt;
    acc_last = acc;
    gyr_last = gyr;
    return true;
  }

  inline bool isInitialized( ) { return flag_init_state_; }

  static void calculateRPfromIMU(const V3D &acc, double &roll, double &pitch)
  {
    pitch = -sign(acc.z( )) * asin(acc.x( ) / G0);
    roll  = sign(acc.z( )) * asin(acc.y( ) / G0);
  }

  void set(const GlobalState &state) { state_ = state; }

  void update(const GlobalState &state,
              const Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_> &covariance)
  {
    state_      = state; // 用迭代后的相对位姿更新滤波器里保存的相对位姿
    covariance_ = covariance; // 更新协方差
  }

  void initialization(double time, const V3D &rn, const V3D &vn, const Q4D &qbn, const V3D &ba, const V3D &bw)
  {
    state_           = GlobalState(rn, vn, qbn, ba, bw);
    time_            = time;
    flag_init_state_ = true;
    initializeCovariance( );
  }

  void initialization(double time, const V3D &rn, const V3D &vn, const Q4D &qbn,
                      const V3D &ba, const V3D &bw,
                      const V3D &acc, const V3D &gyr)
  {
    state_           = GlobalState(rn, vn, qbn, ba, bw);
    time_            = time;
    acc_last         = acc;
    gyr_last         = gyr;
    flag_init_imu_   = true;
    flag_init_state_ = true;
    initializeCovariance( );
  }

  void initialization(double time, const V3D &rn, const V3D &vn, const V3D &ba, const V3D &bw,
                      double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
  {
    state_           = GlobalState(rn, vn, rpy2Quat(V3D(roll, pitch, yaw)), ba, bw);
    time_            = time;
    flag_init_state_ = true;
    initializeCovariance( );
  }

  void initialization(double time, const V3D &rn, const V3D &vn, const V3D &ba, const V3D &bw,
                      const V3D &acc, const V3D &gyr,
                      double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
  {
    state_           = GlobalState(rn, vn, rpy2Quat(V3D(roll, pitch, yaw)), ba, bw);
    time_            = time;
    acc_last         = acc; // 为了中值积分
    gyr_last         = gyr;
    flag_init_imu_   = true;
    flag_init_state_ = true;
    initializeCovariance( );
  }

  ~StatePredictor( ) {}
};

}; // namespace filter


#endif // INCLUDE_KALMANFILTER_HPP_
