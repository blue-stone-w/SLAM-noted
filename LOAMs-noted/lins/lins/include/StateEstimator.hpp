/* done
Description: class: Scan, StateEstimator
Scan: constructor to initialize, reset to clear all, setPointCloud
StateEstimator: contructor to initialize,
                processImu to predict new state and covariance,
                processPCL to process point cloud according to system state,
                undistortPcl & calculateSmoothness,
                rotatePoint to transform the point cloud to the vehicle frame,
                markOccludedPoints to ,
                extractFeatures to mark whether a point has neighbor,
                processFirstScan to initialize the Kalman filter and KD tree

                estimateTransform to match features in two frames and get gobal rotation and position
                estimateInitialState to calculate initial state using IMU preintegration
                updatePointCloud to undistort point cloud using relative transform
                processScan to
                performIESKF to
                solveGyroscopeBias to
                findCorrespondingSurfFeatures.

*/

#ifndef INCLUDE_STATEESTIMATOR_HPP_
#define INCLUDE_STATEESTIMATOR_HPP_

#include <integrationBase.h>
#include <math_utils.h>
#include <parameters.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_broadcaster.h>
#include <tic_toc.h>

#include <KalmanFilter.hpp>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <sensor_utils.hpp>
#include <vector>

#include "cloud_msgs/cloud_info.h"

using namespace Eigen;
using namespace std;
using namespace math_utils;
using namespace sensor_utils;
using namespace parameter;
using namespace filter;

namespace fusion
{

const int LINE_NUM_ = 16; // define again to cover previous value
const int SCAN_NUM_ = 1800;

struct Smooth
{ // Smoothness of point; struct is more effective than class in aspect of arguments
  Smooth( )
  {
    value = 0.0;
    ind   = 0;
  }
  double value;
  size_t ind;
};

struct byValue
{ // compare two nums
  bool operator( )(Smooth const &left, Smooth const &right) { return left.value < right.value; }
};

// Scan Class stores all kinds of information of a point cloud, including
// the whole point cloud, its smoothness, timestamp, and features.
class Scan
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen库为了使用SSE加速，所以内存分配上使用了128位的指针;宏:数据结构内存对齐

      static int scan_counter_; //???
  int id_;
  double time_;

  // Point Cloud
  pcl::PointCloud<PointType>::Ptr distPointCloud_; // original point cloud with distortion
  pcl::PointCloud<PointType>::Ptr undistPointCloud_;
  pcl::PointCloud<PointType>::Ptr outlierPointCloud_;
  cloud_msgs::cloud_info::Ptr cloudInfo_;

  // Pcl Features
  std::vector<double> cloudCurvature_;
  std::vector<Smooth> cloudSmoothness_;

  int cloudNeighborPicked_[LINE_NUM_ * SCAN_NUM_]; // store neighbor of a point
  int cloudLabel_[LINE_NUM_ * SCAN_NUM_];

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp_;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp_;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat_;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat_;

  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharpYZX_; // 使用了不同的坐标系，YZX对应原坐标系的XYZ
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatYZX_;
  pcl::PointCloud<PointType>::Ptr outlierPointCloudYZX_;

  Scan( ) :
    id_(scan_counter_++)
  {
    distPointCloud_.reset(new pcl::PointCloud<PointType>( ));
    undistPointCloud_.reset(new pcl::PointCloud<PointType>( ));
    outlierPointCloud_.reset(new pcl::PointCloud<PointType>( ));
    cornerPointsSharp_.reset(new pcl::PointCloud<PointType>( ));
    cornerPointsLessSharp_.reset(new pcl::PointCloud<PointType>( ));
    surfPointsFlat_.reset(new pcl::PointCloud<PointType>( ));
    surfPointsLessFlat_.reset(new pcl::PointCloud<PointType>( ));
    cloudInfo_.reset(new cloud_msgs::cloud_info( ));

    cornerPointsLessSharpYZX_.reset(new pcl::PointCloud<PointType>( ));
    surfPointsLessFlatYZX_.reset(new pcl::PointCloud<PointType>( ));
    outlierPointCloudYZX_.reset(new pcl::PointCloud<PointType>( ));

    cloudCurvature_.resize(LINE_NUM * SCAN_NUM);
    cloudSmoothness_.resize(LINE_NUM * SCAN_NUM);

    reset( );
  }

  ~Scan( )
  {
    distPointCloud_.reset( );
    undistPointCloud_.reset( );
    outlierPointCloud_.reset( );
    cornerPointsSharp_.reset( );
    cornerPointsLessSharp_.reset( );
    surfPointsFlat_.reset( );
    surfPointsLessFlat_.reset( );
    cloudInfo_.reset( );

    cornerPointsLessSharpYZX_.reset( );
    surfPointsLessFlatYZX_.reset( );
    outlierPointCloudYZX_.reset( );
  }

  void reset( )
  {
    time_ = 0.0;

    distPointCloud_->clear( );
    undistPointCloud_->clear( );
    outlierPointCloud_->clear( );
    cornerPointsSharp_->clear( );
    cornerPointsLessSharp_->clear( );
    surfPointsFlat_->clear( );
    surfPointsLessFlat_->clear( );

    cornerPointsLessSharpYZX_->clear( );
    surfPointsLessFlatYZX_->clear( );
    outlierPointCloudYZX_->clear( );

    cloudCurvature_.assign(LINE_NUM * SCAN_NUM, 0.0);
    cloudSmoothness_.assign(LINE_NUM * SCAN_NUM, Smooth( ));
  }

  void setPointCloud(double time,
                     pcl::PointCloud<PointType>::Ptr distPointCloud,
                     cloud_msgs::cloud_info cloudInfo,
                     pcl::PointCloud<PointType>::Ptr outlierPointCloud)
  {
    distPointCloud_    = distPointCloud;
    *(cloudInfo_)      = cloudInfo;
    outlierPointCloud_ = outlierPointCloud;
    time_              = time;
  }
};
typedef shared_ptr<Scan> ScanPtr; // Define a pointer class for Scan class

// StateEstimator Class implement a iterative-ESKF, including state propagation and update.
class StateEstimator
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum FusionStatus
  {
    STATUS_INIT        = 0,
    STATUS_FIRST_SCAN  = 1,
    STATUS_SECOND_SCAN = 2,
    STATUS_RUNNING     = 3,
    STATUS_RESET       = 4,
  };

  FusionStatus status_; // system status
  StatePredictor *filter_; // Kalman filter pointer
  ScanPtr scan_new_; // current scan information
  ScanPtr scan_last_; // last scan information

  // KD tree relatives
  pcl::VoxelGrid<PointType> downSizeFilter_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCorner_, kdtreeSurf_;

  // Feature matching relatives
  std::vector<int> pointSelCornerInd;
  std::vector<double> pointSearchCornerInd1;
  std::vector<double> pointSearchCornerInd2;
  std::vector<int> pointSelSurfInd;
  std::vector<double> pointSearchSurfInd1;
  std::vector<double> pointSearchSurfInd2;
  std::vector<double> pointSearchSurfInd3;

  // Jacobians and keypoints
  pcl::PointCloud<PointType>::Ptr keypoints_; // key corners and key surf points
  pcl::PointCloud<PointType>::Ptr jacobians_; // jac of key points
  pcl::PointCloud<PointType>::Ptr keypointCorns_;
  pcl::PointCloud<PointType>::Ptr keypointSurfs_;
  pcl::PointCloud<PointType>::Ptr jacobianCoffCorns;
  pcl::PointCloud<PointType>::Ptr jacobianCoffSurfs;

  // Global transformation from the original scan-frame to current scan-frame
  GlobalState globalState_;
  // Relative transformation from scan0-frame t0 scan1-frame
  GlobalState linState_; // 在迭代的过程中linState_是一直被更新的量
  Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, 1> difVecLinInv_; // 对误差状态的估计
  Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, 1> updateVec_; // 每次迭代中的改变量
  double updateVecNorm_ = 0.0; // 每次迭代中改变量的大小

  // Kalman filter relatives
  VXD residual_;
  MXD Fk_, Gk_, Pk_, Qk_, Rk_, Hk_, Jk_, Kk_;
  MXD IKH_, Py_, Pyinv_; // 对应公式（15）

  // IMU preintegration
  integration::IntegrationBase *preintegration_;
  Imu imu_last_;

  // Rotation matrices between XYZ-convention and YZX-convention
  Eigen::Matrix3d R_yzx_to_xyz, R_xyz_to_yzx;
  Eigen::Quaterniond Q_yzx_to_xyz, Q_xyz_to_yzx;
  GlobalState globalStateYZX_;

  StateEstimator( )
  {
    filter_ = new StatePredictor( );

    // Initialize KD tree and downsize filter
    downSizeFilter_.setLeafSize(0.2, 0.2, 0.2);
    kdtreeCorner_.reset(new pcl::KdTreeFLANN<PointType>( ));
    kdtreeSurf_.reset(new pcl::KdTreeFLANN<PointType>( ));
    scan_new_.reset(new Scan( ));
    scan_last_.reset(new Scan( ));

    keypoints_.reset(new pcl::PointCloud<PointType>( ));
    jacobians_.reset(new pcl::PointCloud<PointType>( ));
    keypointCorns_.reset(new pcl::PointCloud<PointType>( ));
    keypointSurfs_.reset(new pcl::PointCloud<PointType>( ));
    jacobianCoffCorns.reset(new pcl::PointCloud<PointType>( ));
    jacobianCoffSurfs.reset(new pcl::PointCloud<PointType>( ));

    surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>( ));
    surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>( ));

    pointSelCornerInd.resize(LINE_NUM * SCAN_NUM);
    pointSearchCornerInd1.resize(LINE_NUM * SCAN_NUM);
    pointSearchCornerInd2.resize(LINE_NUM * SCAN_NUM);
    pointSelSurfInd.resize(LINE_NUM * SCAN_NUM);
    pointSearchSurfInd1.resize(LINE_NUM * SCAN_NUM);
    pointSearchSurfInd2.resize(LINE_NUM * SCAN_NUM);
    pointSearchSurfInd3.resize(LINE_NUM * SCAN_NUM);

    globalState_.setIdentity( );
    globalStateYZX_.setIdentity( );

    // Rotation matrics between XYZ convention and YZX-convention
    R_yzx_to_xyz << 0., 0., 1., 1., 0., 0., 0., 1., 0.; // 是对调，也是一种旋转
    R_xyz_to_yzx = R_yzx_to_xyz.transpose( );
    Q_yzx_to_xyz = R_yzx_to_xyz; // matrix3d可以直接赋值给quaternion
    Q_xyz_to_yzx = R_xyz_to_yzx;

    // gravity_feedback << 0, 0, -G0;

    status_ = STATUS_INIT;
  }

  ~StateEstimator( )
  {
    delete filter_;
    delete preintegration_;
  }

  inline const double getTime( ) const { return filter_->time_; }
  inline bool isInitialized( ) const { return status_ != STATUS_INIT; }

  /********Relative Variables*********/
  V3D pos_;
  V3D vel_;
  M3D quad_; // rotation
  V3D acc_0_;
  V3D gyr_0_;
  /***********************************/
  void processImu(double dt, const V3D &acc, const V3D &gyr)
  {
    switch (status_)
    {
      case STATUS_INIT:
        break;
      case STATUS_FIRST_SCAN:
        preintegration_->push_back(dt, acc, gyr); // data type is pointer of class integration
        filter_->time_ += dt;
        acc_0_ = acc;
        gyr_0_ = gyr;
        break;
      case STATUS_RUNNING:
        filter_->predict(dt, acc, gyr, true); // 计算新的相对位姿，更新误差状态转移矩阵和协方差
        break;
      default:
        break;
    }

    // For no use here. Just propagte IMU measurements for testing
    V3D un_acc_0_ = quad_ * (acc_0_ - INIT_BA) + filter_->state_.gn_;
    V3D un_gyr    = 0.5 * (gyr_0_ + gyr) - INIT_BW;
    quad_ *= math_utils::deltaQ(un_gyr * dt).toRotationMatrix( ); // update rotation
    V3D un_acc_1 = quad_ * (acc - INIT_BA) + filter_->state_.gn_;
    V3D un_acc   = 0.5 * (un_acc_0_ + un_acc_1);
    pos_ += dt * vel_ + 0.5 * dt * dt * un_acc;
    vel_ += dt * un_acc;

    acc_0_ = acc;
    gyr_0_ = gyr;
  }

  /********Relative Variables*********/
  double duration_fea_ = 0; // duration of feature extraction
  double duration_opt_ = 0; // duration of state estimation
  double num_of_edge_  = 0; // num of edge points
  double num_of_surf_  = 0; // num of surf points
  int lidar_counter_   = 0; // 处理的点云的数量
  /***********************************/
  void processPCL(double time, const Imu &imu,
                  pcl::PointCloud<PointType>::Ptr distortedPointCloud,
                  cloud_msgs::cloud_info cloudInfo,
                  pcl::PointCloud<PointType>::Ptr outlierPointCloud)
  {
    TicToc ts_fea; // Calculate the time used in feature extraction
    // 包含了关于该scan所有的中间信息，点云是对指针的复制
    scan_new_->setPointCloud(time, distortedPointCloud, cloudInfo, outlierPointCloud);
    // 将点云从雷达坐标系转换到车体坐标系，并将每个点距离scan第一点的时间戳保存在intensity的小数位中，并没有进行畸变校正
    undistortPcl(scan_new_);
    calculateSmoothness(scan_new_); // 计算曲率
    markOccludedPoints(scan_new_); // 去掉易被遮挡点和噪点
    extractFeatures(scan_new_); // 提取特征，也保存在了scan中
    imu_last_       = imu;
    double time_fea = ts_fea.toc( ); // the time used in feature extraction

    TicToc ts_opt; // Calculate the time used in state estimation
    switch (status_)
    {
      case STATUS_INIT:
        if (processFirstScan( )) { status_ = STATUS_FIRST_SCAN; }
        break;
      case STATUS_FIRST_SCAN:
        if (processSecondScan( )) // 点云只进行了一次初始化，imu进行了预积分
          status_ = STATUS_RUNNING; // 到这里才真正完成了一次初始化
        else
          status_ = STATUS_INIT;
        break;
      case STATUS_RUNNING:
        if (!processScan( )) status_ = STATUS_RUNNING;
        break;
    }
    double time_opt = ts_opt.toc( );

    if (VERBOSE)
    {
      duration_fea_ =
          (duration_fea_ * lidar_counter_ + time_fea) / (lidar_counter_ + 1);
      duration_opt_ =
          (duration_opt_ * lidar_counter_ + time_opt) / (lidar_counter_ + 1);
      num_of_edge_ = (num_of_edge_ * lidar_counter_ + scan_last_->cornerPointsLessSharpYZX_->points.size( )) / (lidar_counter_ + 1);
      num_of_surf_ = (num_of_surf_ * lidar_counter_ + scan_last_->surfPointsLessFlatYZX_->points.size( )) / (lidar_counter_ + 1);
      lidar_counter_++;

      cout << "Feature Extraction: time: " << duration_fea_ << endl;
      cout << "Feature Extraction: corners: " << num_of_edge_
           << ", surfs: " << num_of_surf_ << endl;
      cout << "State Estimation: time: " << duration_opt_ << endl;
    }
  }

  void undistortPcl(ScanPtr scan)
  {
    bool halfPassed = false; // whether this point halfPassed
    scan->undistPointCloud_->clear( );
    pcl::PointCloud<PointType>::Ptr distPointCloud = scan->distPointCloud_;
    cloud_msgs::cloud_info::Ptr segInfo            = scan->cloudInfo_;
    int size                                       = distPointCloud->points.size( );
    PointType point;
    for (int i = 0; i < size; i++)
    {
      // If LiDAR frame does not align with Vehicle frame, we transform the point cloud to the vehicle frame
      rotatePoint(&distPointCloud->points[i], &point);

      double ori = -atan2(point.y, point.x);
      // make sure angle of point is in logical interval
      if (!halfPassed)
      {
        if (ori < segInfo->startOrientation - M_PI / 2)
        {
          ori += 2 * M_PI;
        }
        else if (ori > segInfo->startOrientation + M_PI * 3 / 2)
          ori -= 2 * M_PI;

        if (ori - segInfo->startOrientation > M_PI) halfPassed = true;
      }
      else
      {
        ori += 2 * M_PI;

        if (ori < segInfo->endOrientation - M_PI * 3 / 2)
        {
          ori += 2 * M_PI;
        }
        else if (ori > segInfo->endOrientation + M_PI / 2)
        {
          ori -= 2 * M_PI;
        }
      }
      // assume that laser rotate evenly
      double relTime = (ori - segInfo->startOrientation) / segInfo->orientationDiff;
      // 强度的整数部分是所在行号(在img_project中赋值并发布,Estimator类中订阅,然后传入这个类)
      point.intensity = int(distPointCloud->points[i].intensity) + SCAN_PERIOD * relTime;

      scan->undistPointCloud_->push_back(point);
    } // endfor
  }

  // Coordinate transformation from LiDAR frame to Vehicle frame
  void rotatePoint(PointType const *const pi, PointType *const po)
  {
    V3D rpy;
    rpy << deg2rad(0.0), deg2rad(0.0), deg2rad(IMU_LIDAR_EXTRINSIC_ANGLE);
    M3D R = rpy2R(rpy);
    V3D Pi(pi->x, pi->y, pi->z); // pi is parameter
    V3D Po        = R * Pi; // transformation is only rotation???
    po->x         = Po.x( );
    po->y         = Po.y( );
    po->z         = Po.z( );
    po->intensity = pi->intensity;
  }

  void calculateSmoothness(ScanPtr scan)
  {
    int cloudSize                       = scan->undistPointCloud_->points.size( );
    cloud_msgs::cloud_info::Ptr segInfo = scan->cloudInfo_;
    for (int i = 5; i < cloudSize - 5; i++)
    { // calcute smoothnese
      double diffRange = segInfo->segmentedCloudRange[i - 5] + segInfo->segmentedCloudRange[i - 4] + segInfo->segmentedCloudRange[i - 3] + segInfo->segmentedCloudRange[i - 2] + segInfo->segmentedCloudRange[i - 1] - // here is minus
                         segInfo->segmentedCloudRange[i] * 10 + segInfo->segmentedCloudRange[i + 1] + segInfo->segmentedCloudRange[i + 2] + segInfo->segmentedCloudRange[i + 3] + segInfo->segmentedCloudRange[i + 4] + segInfo->segmentedCloudRange[i + 5];
      scan->cloudCurvature_[i] = diffRange * diffRange; // 需要注意的每行的两端5个点的曲率没有意义

      scan->cloudNeighborPicked_[i] = 0;
      scan->cloudLabel_[i]          = 0;
      // cloudSmoothness for sorting
      scan->cloudSmoothness_[i].value = scan->cloudCurvature_[i];
      scan->cloudSmoothness_[i].ind   = i;
    }
  }

  // according to the article, there are several kinds of points, from which we should't extract features
  // here we mark occluded points and parallel beam points
  void markOccludedPoints(ScanPtr scan)
  {
    int cloudSize                       = scan->undistPointCloud_->points.size( );
    cloud_msgs::cloud_info::Ptr segInfo = scan->cloudInfo_;

    for (int i = 5; i < cloudSize - 6; ++i)
    { // mark occluded points and parallel beam points
      // mark occluded points
      float depth1   = segInfo->segmentedCloudRange[i];
      float depth2   = segInfo->segmentedCloudRange[i + 1];
      int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[i + 1] - segInfo->segmentedCloudColInd[i]));
      if (columnDiff < 10)
      { // 10 pixel diff in range image
        // difference of column is small and difference of range is large
        if (depth1 - depth2 > 0.3)
        {
          // marked as "1", will not be segmented and labeled
          scan->cloudNeighborPicked_[i - 5] = 1;
          scan->cloudNeighborPicked_[i - 4] = 1;
          scan->cloudNeighborPicked_[i - 3] = 1;
          scan->cloudNeighborPicked_[i - 2] = 1;
          scan->cloudNeighborPicked_[i - 1] = 1;
          scan->cloudNeighborPicked_[i]     = 1;
        }
        else if (depth2 - depth1 > 0.3)
        {
          scan->cloudNeighborPicked_[i + 1] = 1;
          scan->cloudNeighborPicked_[i + 2] = 1;
          scan->cloudNeighborPicked_[i + 3] = 1;
          scan->cloudNeighborPicked_[i + 4] = 1;
          scan->cloudNeighborPicked_[i + 5] = 1;
          scan->cloudNeighborPicked_[i + 6] = 1;
        }
      }
      // mark parallel beam
      float diff1 = std::abs(segInfo->segmentedCloudRange[i - 1] - segInfo->segmentedCloudRange[i]);
      float diff2 = std::abs(segInfo->segmentedCloudRange[i + 1] - segInfo->segmentedCloudRange[i]);
      if (diff1 > 0.02 * segInfo->segmentedCloudRange[i] && diff2 > 0.02 * segInfo->segmentedCloudRange[i])
      {
        scan->cloudNeighborPicked_[i] = 1;
      }
    } // endfor
  }

  /********Relative Variables*********/
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS; // DS: down sample
  /***********************************/
  void extractFeatures(ScanPtr scan)
  {
    cloud_msgs::cloud_info::Ptr segInfo = scan->cloudInfo_;
    scan->cornerPointsSharp_->clear( );
    scan->cornerPointsLessSharp_->clear( );
    scan->surfPointsFlat_->clear( );
    scan->surfPointsLessFlat_->clear( );

    for (int i = 0; i < LINE_NUM; i++)
    {
      surfPointsLessFlatScan->clear( );
      // 由于记录了每一行的有意义的起止索引，因此两端5个自然就不计了
      for (int j = 0; j < 6; j++)
      { // devide total cloud into 6 segments
        // start and end point's index
        int sp = (segInfo->startRingIndex[i] * (6 - j) + segInfo->endRingIndex[i] * j) / 6;
        int ep = (segInfo->startRingIndex[i] * (5 - j) + segInfo->endRingIndex[i] * (j + 1)) / 6 - 1;

        if (sp >= ep) continue;

        std::sort(scan->cloudSmoothness_.begin( ) + sp, scan->cloudSmoothness_.begin( ) + ep, byValue( ));

        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--)
        { // find sharp/edge in one single segment
          int ind = scan->cloudSmoothness_[k].ind;
          if (scan->cloudNeighborPicked_[ind] == 0 && // haven't found its neighbor now
              scan->cloudCurvature_[ind] > EDGE_THRESHOLD && // edge feature
              segInfo->segmentedCloudGroundFlag[ind] == false)
          {
            largestPickedNum++;
            if (largestPickedNum <= 2)
            { // select largest 2 points from every segment and save as edge/sharp
              scan->cloudLabel_[ind] = 2;
              scan->cornerPointsSharp_->push_back(scan->undistPointCloud_->points[ind]);
              scan->cornerPointsLessSharp_->push_back(scan->undistPointCloud_->points[ind]);
            }
            else if (largestPickedNum <= 20)
            {
              scan->cloudLabel_[ind] = 1;
              scan->cornerPointsLessSharp_->push_back(scan->undistPointCloud_->points[ind]);
            }
            else
            { // have selected enough edge/sharp points
              break;
            }

            scan->cloudNeighborPicked_[ind] = 1;
            // shouldn't ectract feature point from neighbor area of a feature point
            for (int l = 1; l <= 5; l++)
            {
              int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l - 1]));
              if (columnDiff > 10) break; // means not neighbor area
              scan->cloudNeighborPicked_[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l + 1]));
              if (columnDiff > 10) break;
              scan->cloudNeighborPicked_[ind + l] = 1;
            }
          }
        }

        int smallestPickedNum = 0;
        for (int k = sp; k <= ep; k++)
        { // find surf in one single segment
          int ind = scan->cloudSmoothness_[k].ind;
          if (scan->cloudNeighborPicked_[ind] == 0 && scan->cloudCurvature_[ind] < SURF_THRESHOLD && segInfo->segmentedCloudGroundFlag[ind] == true)
          {
            scan->cloudLabel_[ind] = -1;
            scan->surfPointsFlat_->push_back(scan->undistPointCloud_->points[ind]);
            smallestPickedNum++;
            if (smallestPickedNum >= 4) break;

            scan->cloudNeighborPicked_[ind] = 1;
            for (int l = 1; l <= 5; l++)
            {
              int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l - 1]));
              if (columnDiff > 10) break;

              scan->cloudNeighborPicked_[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int columnDiff = std::abs(int(segInfo->segmentedCloudColInd[ind + l] - segInfo->segmentedCloudColInd[ind + l + 1]));
              if (columnDiff > 10) break;

              scan->cloudNeighborPicked_[ind + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; k++)
        {
          if (scan->cloudLabel_[k] <= 0)
          {
            surfPointsLessFlatScan->push_back(scan->undistPointCloud_->points[k]);
          }
        }
      } // endfor: 已遍历该行
      surfPointsLessFlatScanDS->clear( );
      downSizeFilter_.setInputCloud(surfPointsLessFlatScan);
      downSizeFilter_.filter(*surfPointsLessFlatScanDS);
      *(scan->surfPointsLessFlat_) += *surfPointsLessFlatScanDS;
    } // endfor: 已遍历该点云
    // 每个区域角点最多2+20，面点4+整个line降采样
  }

  // Initialize the Kalman filter and KD tree
  bool processFirstScan( )
  {
    if (scan_new_->cornerPointsLessSharp_->points.size( ) < 10 || scan_new_->surfPointsLessFlat_->points.size( ) < 100)
    {
      ROS_WARN("Wait for more features for initialization...");
      scan_new_.reset(new Scan( ));
      return false;
    }

    // Initialize the Kalman filter
    Fk_.resize(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_);
    Gk_.resize(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_NOISE_);
    Pk_.resize(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_);
    Qk_.resize(GlobalState::DIM_OF_NOISE_, GlobalState::DIM_OF_NOISE_);
    IKH_.resize(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_);
    Fk_.setIdentity( );
    Gk_.setZero( );
    Pk_.setZero( );
    Qk_.setZero( );

    // Set the relative transform to identity
    linState_.setIdentity( ); // data type is GlobalState

    // Initialize IMU preintegration variable
    preintegration_ = new integration::IntegrationBase(imu_last_.acc, imu_last_.gyr, INIT_BA, INIT_BW);

    // Initialize position, velocity, acceleration bias, gyroscope bias by zeros
    // 初始化kalman滤波器的初始姿态、协方差和上一时刻imu测量（TODO:）
    filter_->initialization(scan_new_->time_, V3D(0, 0, 0), V3D(0, 0, 0), // data type: pointer of StatePredictor
                            V3D(0, 0, 0), V3D(0, 0, 0), imu_last_.acc, imu_last_.gyr);

    kdtreeCorner_->setInputCloud(scan_new_->cornerPointsLessSharp_);
    kdtreeSurf_->setInputCloud(scan_new_->surfPointsLessFlat_);

    pos_.setZero( );
    vel_.setZero( );
    quad_.setIdentity( );

    // Slide the point cloud from scan_new_ to scan_last_
    scan_last_.swap(scan_new_); // "swap" can swap any data with same data type;要保证scan_last_的点云成员变量内存不能被释放
    scan_new_.reset(new Scan( ));

    return true;
  }

  // Calculate initial velocity and IMU biases using two consecutive frames and IMU preintegration results
  bool processSecondScan( )
  {
    if (scan_new_->cornerPointsLessSharp_->points.size( ) < 10 || scan_new_->surfPointsLessFlat_->points.size( ) < 100)
    {
      ROS_WARN("Wait for more features for initialization...");
      scan_new_.reset(new Scan( ));
      return false;
    }

    // Calculate relative transform, linState_, using ICP method
    V3D pl;
    Q4D ql;
    V3D v0, v1, ba0, bw0;
    ba0.setZero( );
    ql = preintegration_->delta_q; // 第一帧激光之后，IMU在预积分器中积分的姿态
    pl = preintegration_->delta_p + // 假设两个关键帧之间的重力不变，这里减去重力加速度和bias(内部已经做了，这里设置为0)对位移的影响
         0.5 * linState_.gn_ * preintegration_->sum_dt * preintegration_->sum_dt - 0.5 * ba0 * preintegration_->sum_dt * preintegration_->sum_dt;
    // 上面的linState_.gn_应该是在上一时刻状态下的重力加速度
    // 以上IMU预测了相对姿态和位置，下面以此为初值计算点云相对位姿
    estimateTransform(scan_last_, scan_new_, pl, ql); // 结果会覆盖在pl ql中

    // Calculate initial state using relative transform calculated by point clouds and that by IMU preintegration
    estimateInitialState(pl, ql, v0, v1, ba0, bw0); // 根据相对位置和时间，计算状态量-速度v0v1

    // 由于kalman的状态是相对位姿，因此只有获得了两帧后才能进行kalman的初始化
    // 这里主要是对相对位置和速度等进行了初始化（角度没有用点云匹配的结果，而是单位阵）
    // Initialize the Kalman filter by estimated values
    V3D r1 = pl;
    filter_->initialization(scan_new_->time_, r1, v1, ba0, bw0, imu_last_.acc, imu_last_.gyr);
    double roll_init, pitch_init, yaw_init = deg2rad(0.0);
    // Calculate rough roll and pitch angles using IMU measurements
    // 假设车一开始静止，此时加速度应该为g，根据加速度在xy轴上的分量估计roll和pitch
    // 此处还假设了车辆在开始时处于水平
    calculateRPfromGravity(imu_last_.acc - ba0, roll_init, pitch_init);

    // Initialize the global state, e.g., position, velocity, and orientation represented in the original frame (the first-scan-frame)
    // 初始化状态; 即根据点云匹配计算出的相对位置和速度，静止估计出的roll和pitch（角度是全局的），给定的bias
    globalState_ = GlobalState(r1, v1, rpy2Quat(V3D(roll_init, pitch_init, yaw_init)), ba0, bw0);

    // Use relative transorm linState_ to undistort point cloud (under the constant-speed assumption)
    updatePointCloud( ); // 运动畸变去除，转换到终止时刻，并且交换坐标轴

    scan_last_.swap(scan_new_);
    scan_new_.reset(new Scan( ));

    return true;
  }

  void estimateTransform(ScanPtr lastScan, ScanPtr newScan, V3D &t, Q4D &q)
  {
    double sum_dt  = preintegration_->sum_dt;
    linState_.rn_  = t; // 初始位置和姿态估计
    linState_.qbn_ = q;
    for (int iter = 0; iter < NUM_ITER; iter++)
    {
      keypointSurfs_->clear( );
      jacobianCoffSurfs->clear( );
      keypointCorns_->clear( );
      jacobianCoffCorns->clear( );

      findCorrespondingSurfFeatures(lastScan, newScan, keypointSurfs_, jacobianCoffSurfs, iter);
      if (keypointSurfs_->points.size( ) < 10)
      {
        ROS_WARN("Insufficient matched surfs...");
        continue;
      }
      findCorrespondingCornerFeatures(lastScan, newScan, keypointCorns_, jacobianCoffCorns, iter);
      if (keypointCorns_->points.size( ) < 5)
      {
        ROS_WARN("Insufficient matched corners...");
        continue;
      }

      // 没有用ceres而是构建方程求解变换
      if (calculateTransformation(lastScan, newScan, keypointCorns_, jacobianCoffCorns,
                                  keypointSurfs_, jacobianCoffSurfs, iter))
      {
        ROS_INFO_STREAM("System Converges after " << iter << " iterations");
        break;
      }
    } // endfor: 已完成迭代并得到角度和位置
    t = linState_.rn_;
    q = linState_.qbn_; // qbn_ is quaternion rotation from b-frame to n-frame
  }

  void findCorrespondingSurfFeatures(ScanPtr lastScan, ScanPtr newScan,
                                     pcl::PointCloud<PointType>::Ptr keypoints,
                                     pcl::PointCloud<PointType>::Ptr jacobianCoff,
                                     int iterCount)
  {
    int surfPointsFlatNum = newScan->surfPointsFlat_->points.size( );

    for (int i = 0; i < surfPointsFlatNum; i++)
    {
      PointType pointSel;
      PointType coeff, tripod1, tripod2, tripod3; //, three points to construct a plane
      transformToStart(&newScan->surfPointsFlat_->points[i], &pointSel); // 对优化量进行插值并将点转换到起始时刻
      pcl::PointCloud<PointType>::Ptr laserCloudSurfLast = lastScan->surfPointsLessFlat_;

      // else: 不为该点寻找最近的平面，使用之前的结果
      if (iterCount % ICP_FREQ == 0)
      {
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // (point, num, index of cloest points, distance)
        kdtreeSurf_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
        // closest point, point earlier than closest point, point later
        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

        if (pointSearchSqDis[0] < NEAREST_FEATURE_SEARCH_SQ_DIST)
        {
          closestPointInd = pointSearchInd[0];
          // 强度的整数部分是所在线数; time of closest point
          int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);
          float pointSqDis,
              minPointSqDis2 = NEAREST_FEATURE_SEARCH_SQ_DIST,
              minPointSqDis3 = NEAREST_FEATURE_SEARCH_SQ_DIST;

          for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
          {
            // 根据loam的论文，只在相邻的行中选择最近点
            if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) { break; }

            pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x) + (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y) + (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);
            if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2   = j;
              }
            }
            else
            {
              if (pointSqDis < minPointSqDis3)
              {
                minPointSqDis3 = pointSqDis;
                minPointInd3   = j;
              }
            } // 共线找一个，不共线找一个
          }

          for (int j = closestPointInd - 1; j >= 0; j--)
          {
            if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5)
            {
              break;
            }

            pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x) + (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y) + (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);
            if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2   = j;
              }
            }
            else
            {
              if (pointSqDis < minPointSqDis3)
              {
                minPointSqDis3 = pointSqDis;
                minPointInd3   = j;
              }
            }
          }
        } // (nearest point < NEAREST_FEATURE_SEARCH_SQ_DIST)

        pointSearchSurfInd1[i] = closestPointInd;
        pointSearchSurfInd2[i] = minPointInd2;
        pointSearchSurfInd3[i] = minPointInd3;
      } // (iterCount % ICP_FREQ == 0)

      // 类似于loam求平面方程
      if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0)
      { // if three points are found
        tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
        tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
        tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

        V3D P0xyz(pointSel.x, pointSel.y, pointSel.z);
        V3D P1xyz(tripod1.x, tripod1.y, tripod1.z);
        V3D P2xyz(tripod2.x, tripod2.y, tripod2.z);
        V3D P3xyz(tripod3.x, tripod3.y, tripod3.z);

        V3D M     = math_utils::skew(P1xyz - P2xyz) * (P1xyz - P3xyz); // 叉乘axb=(ax)b，方向是平面的法向,长度是面积
        double r  = (P0xyz - P1xyz).transpose( ) * M; // 点乘(V*norm)
        double m  = M.norm( ); // area of triangle(p1,p2,p3)
        float res = r / m; // 高(误差); distance from point to Corresponding Surf Features

        V3D jacxyz = M.transpose( ) / (m); // normal vector(not unit normal vector); 同时也是残差对xyz的雅可比

        float s = 1; // weight in estimator
        if (iterCount >= ICP_FREQ)
        {
          // 此时为雷达坐标系，距离越远的点的权重越小
          s = 1 - 1.8 * fabs(res) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
        }

        if (s > 0.1 && res != 0)
        {
          coeff.x         = s * jacxyz(0);
          coeff.y         = s * jacxyz(1);
          coeff.z         = s * jacxyz(2);
          coeff.intensity = s * res; // coeffience in estimator

          keypoints->push_back(newScan->surfPointsFlat_->points[i]);
          jacobianCoff->push_back(coeff); // 雅可比和残差，不过这里的雅可比是相对于pointSel的坐标的
        }
      } // (three points are found) and find Correspondence
    } // have found Corresponding Surf Features for every surf point
  }

  // Undistort point cloud to the start frame; 根据优化中量相对位姿和时间戳，将点转换到起始时刻
  void transformToStart(PointType const *const pi, PointType *const po)
  {
    double s = (1.f / SCAN_PERIOD) * (pi->intensity - int(pi->intensity)); // time scale of point in this frame

    V3D P2xyz(pi->x, pi->y, pi->z);
    V3D phi    = Quat2axis(linState_.qbn_); // linState_为需要优化的相对位姿
    Q4D R21xyz = axis2Quat(s * phi);
    R21xyz.normalized( ); //! 这里是bug，应该是R21xyz.normalize();
    V3D T112xyz = s * linState_.rn_;
    V3D P1xyz   = R21xyz * P2xyz + T112xyz;

    po->x         = P1xyz.x( );
    po->y         = P1xyz.y( );
    po->z         = P1xyz.z( );
    po->intensity = pi->intensity;
  }

  void findCorrespondingCornerFeatures(ScanPtr lastScan, ScanPtr newScan,
                                       pcl::PointCloud<PointType>::Ptr keypoints,
                                       pcl::PointCloud<PointType>::Ptr jacobianCoff,
                                       int iterCount)
  {
    int cornerPointsSharpNum = newScan->cornerPointsSharp_->points.size( );
    for (int i = 0; i < cornerPointsSharpNum; i++)
    {
      PointType pointSel;
      PointType coeff, tripod1, tripod2;
      transformToStart(&newScan->cornerPointsSharp_->points[i], &pointSel);
      pcl::PointCloud<PointType>::Ptr laserCloudCornerLast = lastScan->cornerPointsLessSharp_;

      if (iterCount % ICP_FREQ == 0)
      {
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeCorner_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
        int closestPointInd = -1, minPointInd2 = -1;

        if (pointSearchSqDis[0] < NEAREST_FEATURE_SEARCH_SQ_DIST)
        {
          closestPointInd      = pointSearchInd[0];
          int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

          float pointSqDis, minPointSqDis2 = NEAREST_FEATURE_SEARCH_SQ_DIST;

          for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
          {
            if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5)
            {
              break;
            }

            pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * (laserCloudCornerLast->points[j].x - pointSel.x) + (laserCloudCornerLast->points[j].y - pointSel.y) * (laserCloudCornerLast->points[j].y - pointSel.y) + (laserCloudCornerLast->points[j].z - pointSel.z) * (laserCloudCornerLast->points[j].z - pointSel.z);
            if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2   = j;
              }
            }
          }

          for (int j = closestPointInd - 1; j >= 0; j--)
          {
            if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5)
            {
              break;
            }

            pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * (laserCloudCornerLast->points[j].x - pointSel.x) + (laserCloudCornerLast->points[j].y - pointSel.y) * (laserCloudCornerLast->points[j].y - pointSel.y) + (laserCloudCornerLast->points[j].z - pointSel.z) * (laserCloudCornerLast->points[j].z - pointSel.z);

            if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan)
            {
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2   = j;
              }
            }
          }
        }
        pointSearchCornerInd1[i] = closestPointInd;
        pointSearchCornerInd2[i] = minPointInd2;
      }

      if (pointSearchCornerInd2[i] >= 0)
      {
        tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
        tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

        V3D P0xyz(pointSel.x, pointSel.y, pointSel.z);
        V3D P1xyz(tripod1.x, tripod1.y, tripod1.z);
        V3D P2xyz(tripod2.x, tripod2.y, tripod2.z);

        V3D P     = math_utils::skew(P0xyz - P1xyz) * (P0xyz - P2xyz); // 面积法向量
        float r   = P.norm( ); // the area of this triangle
        float d12 = (P1xyz - P2xyz).norm( ); // distance of two points
        float res = r / d12; // the heignt of this triangle

        V3D jacxyz = P.transpose( ) * math_utils::skew(P2xyz - P1xyz) / (d12 * r); // 单位高向量, direction of jac

        float s = 1;

        if (iterCount >= ICP_FREQ)
        {
          s = 1 - 1.8 * fabs(res);
        }

        if (s > 0.1 && res != 0)
        {
          coeff.x         = s * jacxyz(0);
          coeff.y         = s * jacxyz(1);
          coeff.z         = s * jacxyz(2);
          coeff.intensity = s * res;

          keypoints->push_back(newScan->cornerPointsSharp_->points[i]);
          jacobianCoff->push_back(coeff);
        }
      }
    }
  }

  bool calculateTransformation(ScanPtr lastScan, ScanPtr newScan,
                               pcl::PointCloud<PointType>::Ptr corners,
                               pcl::PointCloud<PointType>::Ptr jacoCornersCoff,
                               pcl::PointCloud<PointType>::Ptr surfs,
                               pcl::PointCloud<PointType>::Ptr jacoSurfsCoff,
                               int iterCount)
  {
    keypoints_->clear( );
    jacobians_->clear( );
    (*keypoints_) += (*surfs);
    (*keypoints_) += (*corners);
    (*jacobians_) += (*jacoSurfsCoff);
    (*jacobians_) += (*jacoCornersCoff);

    const int stateNum = 6; // 3(rotation)+3(position)
    const int pointNum = keypoints_->points.size( );
    const int imuNum   = 0;
    const int row      = pointNum + imuNum;
    Eigen::Matrix<double, Eigen::Dynamic, stateNum> J(row, stateNum);
    Eigen::Matrix<double, stateNum, Eigen::Dynamic> JT(stateNum, row);
    Eigen::Matrix<double, stateNum, stateNum> JTJ;
    Eigen::VectorXd b(row);
    Eigen::Matrix<double, stateNum, 1> JTb;
    Eigen::Matrix<double, stateNum, 1> x; // error-state
    J.setZero( );
    JT.setZero( );
    JTJ.setZero( );
    b.setZero( );
    JTb.setZero( );
    x.setZero( );

    for (int i = 0; i < pointNum; ++i)
    { // get residual for every point
      // Select keypoint i
      const PointType &keypoint = keypoints_->points[i];
      const PointType &coeff    = jacobians_->points[i];
      V3D P2xyz(keypoint.x, keypoint.y, keypoint.z);
      V3D coff_xyz(coeff.x, coeff.y, coeff.z);

      double s = (1.f / SCAN_PERIOD) * (keypoint.intensity - int(keypoint.intensity));

      V3D phi = Quat2axis(linState_.qbn_);
      // Rotation matrix from frame2 (new) to frame1 (last)
      Q4D R21xyz = axis2Quat(s * phi);
      R21xyz.normalized( );
      // Translation vector from frame1 to frame2 represented in frame1
      V3D T112xyz = s * linState_.rn_;
      // t2=r*t1+t
      V3D jacobian1xyz = coff_xyz.transpose( ) * (-R21xyz.toRotationMatrix( ) * skew(P2xyz)); // rotation jacobian
      V3D jacobian2xyz = coff_xyz.transpose( ) * M3D::Identity( ); // translation jacobian
      double residual  = coeff.intensity;

      J.block<1, 3>(i, O_R) = jacobian1xyz; // declare in parameters.h
      J.block<1, 3>(i, O_P) = jacobian2xyz;

      // Set the overall residual
      b(i) = -0.05 * residual; // 控制每次迭代的步进量
    }

    // Solve x
    JT  = J.transpose( );
    JTJ = JT * J;
    JTb = JT * b;
    x   = JTJ.colPivHouseholderQr( ).solve(JTb); // 求解最小二乘，Ax = b -> x=A.colPivHouseholderQr().solve(b)

    // Determine whether x is degenerated
    bool isDegenerate = false;
    Eigen::Matrix<double, stateNum, stateNum> matP;
    if (iterCount == 0)
    {
      Eigen::Matrix<double, 1, stateNum> matE; // eigenvalues
      Eigen::Matrix<double, stateNum, stateNum> matV; // eigenvectors
      Eigen::Matrix<double, stateNum, stateNum> matV2; // to store data in mat V temporarily

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, stateNum, stateNum>> esolver(JTJ);
      matE = esolver.eigenvalues( ).real( );
      matV = esolver.eigenvectors( ).real( );

      matV2 = matV;

      isDegenerate = false;
      std::vector<double> eignThre(stateNum, 10.); // size=6, value=10
      for (int i = 0; i < stateNum; i++)
      { // traverse 6 state: 3(rotation) +3(position)
        // if eigenvalue is less than 10, set the corresponding eigenvector to 0 vector
        if (matE(0, i) < eignThre[i])
        {
          for (int j = 0; j < stateNum; j++)
          {
            matV2(i, j) = 0;
          }
          isDegenerate = true;
        }
        else
        {
          break;
        }
      }
      matP = matV.inverse( ) * matV2;
    }

    if (isDegenerate)
    { // set degenerated component of delta x as zero
      cout << "System is Degenerate." << endl;
      Eigen::Matrix<double, stateNum, 1> matX2(x);
      x = matP * matX2;
    }

    // Update state linState_
    Q4D dq         = rpy2Quat(x.segment<3>(O_R)); // select 3 elements from O_R_th element
    linState_.qbn_ = (linState_.qbn_ * dq).normalized( );
    linState_.rn_ += x.segment<3>(O_P);

    // Determine whether should it stop
    V3D rpy_rad   = x.segment<3>(O_R);
    V3D rpy_deg   = math_utils::rad2deg(rpy_rad);
    double deltaR = rpy_deg.norm( );
    V3D trans     = 100 * x.segment<3>(O_P); // unit: centimeter
    double deltaT = trans.norm( );
    if (deltaR < 0.1 && deltaT < 0.1) return true;

    return false;
  }

  void estimateInitialState(const V3D &p, const Q4D &q, V3D &v0, V3D &v1, V3D &ba, V3D &bw)
  {
    double sum_dt = preintegration_->sum_dt;
    // Calculate a rough velocity using relative translation
    V3D v = p / sum_dt;
    v0    = v;
    v1    = v;
    // TODO: calibrate initial ba and bw using two consecutive scans and IMU preintegration results
    ba = INIT_BA;
    bw = INIT_BW;
  }

  // get v0, v1; initialize ba and bw with INIT_BA and INIT_BW
  void estimateInitialState1(const V3D &p, const Q4D &q, V3D &v0, V3D &v1, V3D &ba, V3D &bw)
  {
    ba = INIT_BA;
    bw = INIT_BW;
    solveGyroscopeBias(q, bw);

    double sum_dt = preintegration_->sum_dt;

    v0 = (p - 0.5 * linState_.gn_ * sum_dt * sum_dt - preintegration_->delta_p) / sum_dt;
    v1 = v0 + sum_dt * linState_.gn_ + preintegration_->delta_v;

    cout << "v0: " << v0.transpose( ) << endl;
    cout << "v1: " << v1.transpose( ) << endl;
    cout << "ba0: " << INIT_BA.transpose( ) << endl;
    cout << "bw0: " << INIT_BW.transpose( ) << endl;
    cout << "bw0: " << bw.transpose( ) << endl;
  }

  // get v0, v1???; initialize ba and bw with INIT_BA and INIT_BW
  void estimateInitialState2(const V3D &p, const Q4D &q, V3D &v0, V3D &v1, V3D &ba, V3D &bw)
  {
    const int DIM_OF_STATE = 1 + 1 + 3; //???
    const int DIM_OF_MEAS  = 3 + 3;
    Eigen::Matrix<double, Eigen::Dynamic, DIM_OF_STATE> J(DIM_OF_MEAS, DIM_OF_STATE);
    Eigen::Matrix<double, DIM_OF_STATE, Eigen::Dynamic> JT(DIM_OF_STATE, DIM_OF_MEAS);

    Eigen::Matrix<double, DIM_OF_STATE, DIM_OF_STATE> JTJ;
    Eigen::VectorXd b(DIM_OF_MEAS); //
    Eigen::Matrix<double, DIM_OF_STATE, 1> JTb;
    Eigen::Matrix<double, DIM_OF_STATE, 1> x; // error-state

    J.setZero( );
    JT.setZero( );
    JTJ.setZero( );
    b.setZero( );
    JTb.setZero( );
    x.setZero( );

    double sum_dt       = preintegration_->sum_dt;
    b.block<3, 1>(0, 0) = (p - preintegration_->delta_p) / sum_dt - 0.5 * sum_dt * linState_.gn_;
    b.block<3, 1>(3, 0) = sum_dt * linState_.gn_ + preintegration_->delta_v;

    V3D L(1, 0, 0);
    J.block<3, 1>(0, 0) = L; //???
    J.block<3, 3>(0, 2) = -0.5 * sum_dt * M3D::Identity( );
    J.block<3, 1>(3, 0) = -L;
    J.block<3, 1>(3, 1) = L;
    J.block<3, 3>(3, 2) = sum_dt * M3D::Identity( );

    JT  = J.transpose( );
    JTJ = JT * J;
    JTb = JT * b;

    x  = JTJ.colPivHouseholderQr( ).solve(JTb);
    v0 = x(0) * L; //???
    v1 = x(1) * L;
    ba = INIT_BA;
    bw = INIT_BW;

    V3D test_ba = linState_.gn_ + preintegration_->delta_v / sum_dt; //???
    cout << "test_ba: " << test_ba.transpose( ) << endl;
  }

  // in constant velocity
  void estimateInitialState3(const V3D &p, const Q4D &q, V3D &v0, V3D &v1, V3D &ba, V3D &bw)
  {
    double sum_dt = preintegration_->sum_dt;
    V3D v         = p / sum_dt;
    // v * sum_dt = (p - 0.5*linState_.gn_*sum_dt*sum_dt - preintegration_->delta_p + 0.5*ba*sum_dt*sum_dt);
    ba = (v * sum_dt - p + 0.5 * linState_.gn_ * sum_dt * sum_dt + preintegration_->delta_p) * 2 * (1.0 / sum_dt * sum_dt);

    solveGyroscopeBias(q, bw);

    v0 = v;
    v1 = v;
    cout << "v0: " << v0.transpose( ) << endl;
    cout << "v1: " << v1.transpose( ) << endl;
    cout << "ba0: " << ba.transpose( ) << endl;
    cout << "bw0: " << bw.transpose( ) << endl;
  }

  // (acceleration-bias, roll, pitch)
  void calculateRPfromGravity(const V3D &fbib, double &roll, double &pitch)
  {
    pitch = -sign(fbib.z( )) * asin(fbib.x( ) / G0);
    roll  = sign(fbib.z( )) * asin(fbib.y( ) / G0);
  }

  // Undistort point cloud using relative transform
  // 点云校正到终止时刻，对点云和当前全局位姿进行坐标系交换（可能是为了使用mapping模块）
  void updatePointCloud( )
  {
    scan_new_->cornerPointsLessSharpYZX_->clear( );
    scan_new_->surfPointsLessFlatYZX_->clear( );
    scan_new_->outlierPointCloudYZX_->clear( );

    PointType point;
    for (int i = 0; i < scan_new_->cornerPointsLessSharp_->points.size( ); i++)
    {
      transformToEnd(&scan_new_->cornerPointsLessSharp_->points[i],
                     &scan_new_->cornerPointsLessSharp_->points[i]);
      // Transform XYZ-convention to YZX-convention to meet the mapping module's requirement
      point.x         = scan_new_->cornerPointsLessSharp_->points[i].y;
      point.y         = scan_new_->cornerPointsLessSharp_->points[i].z;
      point.z         = scan_new_->cornerPointsLessSharp_->points[i].x;
      point.intensity = scan_new_->cornerPointsLessSharp_->points[i].intensity;
      scan_new_->cornerPointsLessSharpYZX_->push_back(point);
    }
    for (int i = 0; i < scan_new_->surfPointsLessFlat_->points.size( ); i++)
    {
      transformToEnd(&scan_new_->surfPointsLessFlat_->points[i],
                     &scan_new_->surfPointsLessFlat_->points[i]);
      point.x         = scan_new_->surfPointsLessFlat_->points[i].y;
      point.y         = scan_new_->surfPointsLessFlat_->points[i].z;
      point.z         = scan_new_->surfPointsLessFlat_->points[i].x;
      point.intensity = scan_new_->surfPointsLessFlat_->points[i].intensity;
      scan_new_->surfPointsLessFlatYZX_->push_back(point);
    }
    for (int i = 0; i < scan_new_->outlierPointCloud_->points.size( ); i++)
    {
      // transformToEnd(&scan_new_->outlierPointCloud_->points[i],
      //                &scan_new_->outlierPointCloud_->points[i]);
      point.x         = scan_new_->outlierPointCloud_->points[i].y;
      point.y         = scan_new_->outlierPointCloud_->points[i].z;
      point.z         = scan_new_->outlierPointCloud_->points[i].x;
      point.intensity = scan_new_->outlierPointCloud_->points[i].intensity;
      scan_new_->outlierPointCloudYZX_->push_back(point);
    }

    // Transform XYZ-convention to YZX-convention to meet the mapping module's requirement
    globalStateYZX_.rn_  = Q_xyz_to_yzx * globalState_.rn_;
    globalStateYZX_.qbn_ = Q_xyz_to_yzx * globalState_.qbn_ * Q_xyz_to_yzx.inverse( );

    if (scan_new_->cornerPointsLessSharp_->points.size( ) >= 5 && scan_new_->surfPointsLessFlat_->points.size( ) >= 20)
    {
      kdtreeCorner_->setInputCloud(scan_new_->cornerPointsLessSharp_);
      kdtreeSurf_->setInputCloud(scan_new_->surfPointsLessFlat_);
    }
  }

  // Undistort point cloud to the end frame
  void transformToEnd(PointType const *const pi, PointType *const po)
  {
    double s = (1.f / SCAN_PERIOD) * (pi->intensity - int(pi->intensity));

    V3D P2xyz(pi->x, pi->y, pi->z);
    V3D phi    = Quat2axis(linState_.qbn_);
    Q4D R21xyz = axis2Quat(s * phi);
    R21xyz.normalized( );
    V3D T112xyz = s * linState_.rn_;
    V3D P1xyz   = R21xyz * P2xyz + T112xyz;

    R21xyz  = linState_.qbn_;
    T112xyz = linState_.rn_;
    P2xyz   = R21xyz.inverse( ) * (P1xyz - T112xyz);

    po->x         = P2xyz.x( );
    po->y         = P2xyz.y( );
    po->z         = P2xyz.z( );
    po->intensity = pi->intensity;
  }

  bool processScan( )
  {
    if (scan_new_->cornerPointsLessSharp_->points.size( ) <= 5 || scan_new_->surfPointsLessFlat_->points.size( ) <= 10)
    {
      ROS_WARN("Insufficient features...State estimation fails.");
      return false;
    }

    // Update states; 迭代对相对位置和误差状态协方差进行迭代更新，保存在linState_和Pk_中，并update到滤波器中
    performIESKF( );
    // Update global transform by estimated relative transform; 相对姿态累积成全局姿态
    integrateTransformation( );
    filter_->reset(1);

    // 根据滤波器得到的当前坐标系下重力的分量，重新计算roll和pitch，更新到全局姿态
    double roll, pitch;
    // Because the estimated gravity is represented in the b-frame, we can directly
    // solve more accurate roll and pitch angles to correct the global state
    correctRollPitch(roll, pitch);

    // 点云校正到终止时刻，并对点云和绝对位姿进行坐标习惯变换
    // Undistort point cloud using estimated relative transform
    updatePointCloud( );

    // Slide the new scan to last scan
    scan_last_.swap(scan_new_);
    scan_new_.reset(new Scan( ));

    return true;
  }

  // 对相对姿态和误差状态协方差进行迭代更新，结果都是相对于上一关键帧的
  void performIESKF( )
  {
    // Store current state and perform initialization
    Pk_                     = filter_->covariance_; // 使用IMU一直更新的相对位姿误差协方差矩阵
    GlobalState filterState = filter_->state_; // 相对上一关键帧的位姿
    linState_               = filterState; // linState_是对点云相对变换的预测，会一直被更新

    double residualNorm             = 1e6; // infinite number
    bool hasConverged               = false;
    bool hasDiverged                = false;
    const unsigned int DIM_OF_STATE = GlobalState::DIM_OF_STATE_;
    for (int iter = 0; iter < NUM_ITER && !hasConverged && !hasDiverged; iter++)
    {
      keypointSurfs_->clear( );
      jacobianCoffSurfs->clear( );
      keypointCorns_->clear( );
      jacobianCoffCorns->clear( );

      // Find corresponding features
      findCorrespondingSurfFeatures(scan_last_, scan_new_, keypointSurfs_, jacobianCoffSurfs, iter);
      if (keypointSurfs_->points.size( ) < 10)
      {
        if (VERBOSE)
        {
          ROS_WARN("Insufficient matched surfs...");
        }
      }
      findCorrespondingCornerFeatures(scan_last_, scan_new_, keypointCorns_, jacobianCoffCorns, iter);
      if (keypointCorns_->points.size( ) < 5)
      {
        if (VERBOSE)
        {
          ROS_WARN("Insufficient matched corners...");
        }
      }
      // 以上对每个特征点投影到起始时刻并搜索最近邻，计算残差和雅可比（残差是相对于投影后的点的坐标的）

      // Sum up jocobians and residuals
      keypoints_->clear( );
      jacobians_->clear( );
      (*keypoints_) += (*keypointSurfs_);
      (*keypoints_) += (*keypointCorns_);
      (*jacobians_) += (*jacobianCoffSurfs);
      (*jacobians_) += (*jacobianCoffCorns);

      // Memery allocation
      const unsigned int DIM_OF_MEAS = keypoints_->points.size( );
      residual_.resize(DIM_OF_MEAS);
      Hk_.resize(DIM_OF_MEAS, DIM_OF_STATE);
      Rk_.resize(DIM_OF_MEAS, DIM_OF_MEAS);
      Kk_.resize(DIM_OF_STATE, DIM_OF_MEAS);
      Py_.resize(DIM_OF_MEAS, DIM_OF_MEAS);
      Pyinv_.resize(DIM_OF_MEAS, DIM_OF_MEAS);

      Hk_.setZero( ); // set the measurement matrix
      V3D axis = Quat2axis(linState_.qbn_);
      for (int i = 0; i < DIM_OF_MEAS; ++i)
      { // num of points
        // Point represented in 2-frame (e.g., the end frame) in a xyz-convention
        // coordinate of point
        V3D P2xyz(keypoints_->points[i].x, keypoints_->points[i].y, keypoints_->points[i].z);
        // normal vector from feature to point
        V3D coff_xyz(jacobians_->points[i].x, jacobians_->points[i].y, jacobians_->points[i].z);
        residual_(i) = LIDAR_SCALE * jacobians_->points[i].intensity; // 给残差的系数可以用于调整每次迭代的步长
        // TODO:以下求解雅可比的详细推导还没有搞清楚
        // 在迭代的过程中linState_是一直被更新的量
        Hk_.block<1, 3>(i, GlobalState::att_) = coff_xyz.transpose( ) * (-linState_.qbn_.toRotationMatrix( ) * skew(P2xyz)) * Rinvleft(-axis); //???
        Hk_.block<1, 3>(i, GlobalState::pos_) = coff_xyz.transpose( ) * M3D::Identity( ); // TODO:是不是少了一个姿态外参的逆和系数s
      }

      // Set the measurement covariance matrix
      VXD cov = VXD::Zero(DIM_OF_MEAS);
      for (int i = 0; i < DIM_OF_MEAS; ++i)
      {
        cov[i] = LIDAR_STD * LIDAR_STD;
      }
      Rk_ = cov.asDiagonal( );

      // Kalman filter update. Details can be referred to ROVIO
      // 对应公式（15）
      // Pk_为误差状态协方差，是使用IMU积分得到的，这里进行的是迭代更新
      Py_ = Hk_ * Pk_ * Hk_.transpose( ) + Rk_; // S = H * P * H.transpose() + R;
      Pyinv_.setIdentity( ); // pyinv_ is unit matrix
      Py_.llt( ).solveInPlace(Pyinv_); // solve Ax=B, to get inverse
      Kk_ = Pk_ * Hk_.transpose( ) * Pyinv_; // K = P*H.transpose()*S.inverse()

      // linState_相当于是一直被更新的对相对姿态更好的估计。
      // 新状态与纯IMU预测filterState相比，之间的差就是对误差的估计（即预测应该被修正的量，一开始为0）
      // filterState是通过IMU对相对位姿的预测，difVecLinInv_是对误差状态的估计
      filterState.boxMinus(linState_, difVecLinInv_); // difVecLinInv_ is output
      // 对误差进行更新，对应论文公式（16-17）
      updateVec_ = -Kk_ * (residual_ + Hk_ * difVecLinInv_) + difVecLinInv_;

      // 以上对位姿误差状态进行了迭代更新
      // 但是没有对误差状态协方差进行更新（在下面进行）

      // Divergence determination
      bool hasNaN = false;
      for (int i = 0; i < updateVec_.size( ); i++)
      {
        if (isnan(updateVec_[i]))
        {
          updateVec_[i] = 0;
          hasNaN        = true;
        }
      }
      if (hasNaN == true)
      {
        ROS_WARN("System diverges Because of NaN...");
        hasDiverged = true;
        break;
      }

      // Check whether the filter converges
      if (residual_.norm( ) > residualNorm * 10)
      {
        ROS_WARN("System diverges...");
        hasDiverged = true;
        break;
      }

      // Update the state; 对应论文公式（4）
      linState_.boxPlus(updateVec_, linState_);

      updateVecNorm_ = updateVec_.norm( );
      if (updateVecNorm_ <= 1e-2) hasConverged = true;
      residualNorm = residual_.norm( );
    } // endfor: 状态linState_已经被更新成最优，误差状态updateVec_也已经最优

    // If diverges, switch to traditional ICP method to get a rough relative
    // transformation. Otherwise, update the error-state covariance matrix
    if (hasDiverged == true)
    {
      ROS_WARN("======Using ICP Method======");
      V3D t = filterState.rn_;
      Q4D q = filterState.qbn_;
      estimateTransform(scan_last_, scan_new_, t, q);
      filterState.rn_  = t;
      filterState.qbn_ = q;
      filter_->update(filterState, Pk_); // 更新状态和误差状态协方差
    }
    else
    {
      // Update only one time; 公式(18)
      IKH_ = Eigen::Matrix<double, 18, 18>::Identity( ) - Kk_ * Hk_;
      Pk_  = IKH_ * Pk_ * IKH_.transpose( ) + Kk_ * Rk_ * Kk_.transpose( ); // 对相对姿态误差协方差进行更新
      enforceSymmetry(Pk_);
      filter_->update(linState_, Pk_); // 用迭代出来的最新位姿和误差方差更新滤波器，但是都是在上一关键帧坐标系下的
    }
  }

  // 对全局位置，姿态，速度，重力，bias进行更新，其中bias不涉及坐标系变换
  // Update the gloabl state by the new relative transformation
  void integrateTransformation( )
  {
    GlobalState filterState = filter_->state_; // 相对位姿状态
    globalState_.rn_        = globalState_.qbn_ * filterState.rn_ + globalState_.rn_;
    globalState_.qbn_       = globalState_.qbn_ * filterState.qbn_; // 注意此时globalState_.qbn_已经更新到当前时刻了
    globalState_.vn_        = globalState_.qbn_ * filterState.qbn_.inverse( ) * filterState.vn_;
    globalState_.ba_        = filterState.ba_;
    globalState_.bw_        = filterState.bw_;
    globalState_.gn_        = globalState_.qbn_ * filterState.gn_; // TODO:这里应该缺一个filterState.qbn_.inverse()吧
  }

  // roll and pitch is calculated by gravity, yaw is not estimated
  void correctRollPitch(const double &roll, const double &pitch)
  {
    V3D rpy           = math_utils::Q2rpy(globalState_.qbn_);
    Q4D quad          = math_utils::rpy2Quat(V3D(roll, pitch, rpy[2]));
    globalState_.qbn_ = quad;
  }

  void correctOrientation(const Q4D &quad) { globalState_.qbn_ = quad; }

  // Estimate gyroscope bias using a similar method provided in VINS-Mono
  void solveGyroscopeBias(const Q4D &q, V3D &bw)
  {
    Matrix3d A; // Jacobian
    V3D b;
    V3D delta_bg; // correction of bias of g
    A.setZero( );
    b.setZero( );

    MatrixXd tmp_A(3, 3);
    tmp_A.setZero( );
    VectorXd tmp_b(3);
    tmp_b.setZero( );
    Eigen::Quaterniond q_ij = q;
    tmp_A                   = preintegration_->jacobian.template block<3, 3>(GlobalState::att_, GlobalState::gyr_);
    tmp_b                   = 2 * (preintegration_->delta_q.inverse( ) * q_ij).vec( );
    A += tmp_A.transpose( ) * tmp_A;
    b += tmp_A.transpose( ) * tmp_b;

    delta_bg = A.ldlt( ).solve(b); // solve Ax = b||time*delta_bg=measurement.q-preintegration.q
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose( ));

    bw += delta_bg;
  }
};

} // namespace fusion

#endif // INCLUDE_STATEESTIMATOR_HPP_
