// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.

// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"

class FeatureAssociation
{
 private:
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud;
  ros::Subscriber subLaserCloudInfo;
  ros::Subscriber subOutlierCloud;
  ros::Subscriber subImu;

  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;
  double timeNewSegmentedCloud;
  double timeNewSegmentedCloudInfo;
  double timeNewOutlierCloud;

  bool newSegmentedCloud;
  bool newSegmentedCloudInfo;
  bool newOutlierCloud;

  cloud_msgs::cloud_info segInfo;
  std_msgs::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  float *cloudCurvature;
  int *cloudNeighborPicked;
  int *cloudLabel;

  int imuPointerFront; // 比该点晚的IMU数据中最早的一帧
  int imuPointerLast;  // ???
  int imuPointerLastIteration;

  float imuRollStart, imuPitchStart, imuYawStart;
  float cosImuRollStart, cosImuPitchStart, cosImuYawStart; //???
  float sinImuRollStart, sinImuPitchStart, sinImuYawStart; //???
  float imuRollCur, imuPitchCur, imuYawCur;

  float imuVeloXStart, imuVeloYStart, imuVeloZStart;    // 该帧点云的起始点的速度
  float imuShiftXStart, imuShiftYStart, imuShiftZStart; // 该帧点云的起始点的位移

  float imuVeloXCur, imuVeloYCur, imuVeloZCur;    // 当前正在处理的点的速度
  float imuShiftXCur, imuShiftYCur, imuShiftZCur; // 当前正在处理的点的位移

  float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
  float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

  // 当前帧点云起始时刻的角度
  float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;
  // 上一帧点云起始时刻的角度
  float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;
  // 当前帧和上一帧起始时刻的角度差
  float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;

  double imuTime[imuQueLength];
  float imuRoll[imuQueLength];
  float imuPitch[imuQueLength];
  float imuYaw[imuQueLength];

  float imuAccX[imuQueLength];
  float imuAccY[imuQueLength];
  float imuAccZ[imuQueLength];

  float imuVeloX[imuQueLength];
  float imuVeloY[imuQueLength];
  float imuVeloZ[imuQueLength];

  float imuShiftX[imuQueLength];
  float imuShiftY[imuQueLength];
  float imuShiftZ[imuQueLength];

  float imuAngularVeloX[imuQueLength];
  float imuAngularVeloY[imuQueLength];
  float imuAngularVeloZ[imuQueLength];

  float imuAngularRotationX[imuQueLength];
  float imuAngularRotationY[imuQueLength];
  float imuAngularRotationZ[imuQueLength];

  ros::Publisher pubLaserCloudCornerLast;
  ros::Publisher pubLaserCloudSurfLast;
  ros::Publisher pubLaserOdometry;
  ros::Publisher pubOutlierCloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  int *pointSelCornerInd;
  float *pointSearchCornerInd1;
  float *pointSearchCornerInd2;

  int *pointSelSurfInd;
  float *pointSearchSurfInd1;
  float *pointSearchSurfInd2;
  float *pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  float imuRollLast, imuPitchLast, imuYawLast;
  float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
  float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri; // feature point for association
  pcl::PointCloud<PointType>::Ptr coeffSel;      // jacobi and residual

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

  nav_msgs::Odometry laserOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;

  bool isDegenerate;
  cv::Mat matP;

  int frameCount;

 public:
  FeatureAssociation() :
    nh("~")
  {
    subLaserCloud     = nh.subscribe<sensor_msgs::PointCloud2>("/segmented_cloud", 1, &FeatureAssociation::laserCloudHandler, this);
    subLaserCloudInfo = nh.subscribe<cloud_msgs::cloud_info>("/segmented_cloud_info", 1, &FeatureAssociation::laserCloudInfoHandler, this);
    subOutlierCloud   = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud", 1, &FeatureAssociation::outlierCloudHandler, this);
    subImu            = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &FeatureAssociation::imuHandler, this);

    pubCornerPointsSharp     = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
    pubSurfPointsFlat        = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
    pubSurfPointsLessFlat    = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

    pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
    pubLaserCloudSurfLast   = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
    pubOutlierCloudLast     = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
    pubLaserOdometry        = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);

    initializationValue();
  }

  void initializationValue()
  {
    cloudCurvature      = new float[N_SCAN * Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
    cloudLabel          = new int[N_SCAN * Horizon_SCAN];

    pointSelCornerInd     = new int[N_SCAN * Horizon_SCAN];
    pointSearchCornerInd1 = new float[N_SCAN * Horizon_SCAN];
    pointSearchCornerInd2 = new float[N_SCAN * Horizon_SCAN];

    pointSelSurfInd     = new int[N_SCAN * Horizon_SCAN];
    pointSearchSurfInd1 = new float[N_SCAN * Horizon_SCAN];
    pointSearchSurfInd2 = new float[N_SCAN * Horizon_SCAN];
    pointSearchSurfInd3 = new float[N_SCAN * Horizon_SCAN];

    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
    cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
    surfPointsFlat.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

    surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

    timeScanCur               = 0;
    timeNewSegmentedCloud     = 0;
    timeNewSegmentedCloudInfo = 0;
    timeNewOutlierCloud       = 0;

    newSegmentedCloud     = false;
    newSegmentedCloudInfo = false;
    newOutlierCloud       = false;

    systemInitCount = 0;
    systemInited    = false;

    imuPointerFront         = 0;
    imuPointerLast          = -1;
    imuPointerLastIteration = 0;

    imuRollStart     = 0;
    imuPitchStart    = 0;
    imuYawStart      = 0;
    cosImuRollStart  = 0;
    cosImuPitchStart = 0;
    cosImuYawStart   = 0;
    sinImuRollStart  = 0;
    sinImuPitchStart = 0;
    sinImuYawStart   = 0;
    imuRollCur       = 0;
    imuPitchCur      = 0;
    imuYawCur        = 0;

    imuVeloXStart  = 0;
    imuVeloYStart  = 0;
    imuVeloZStart  = 0;
    imuShiftXStart = 0;
    imuShiftYStart = 0;
    imuShiftZStart = 0;

    imuVeloXCur  = 0;
    imuVeloYCur  = 0;
    imuVeloZCur  = 0;
    imuShiftXCur = 0;
    imuShiftYCur = 0;
    imuShiftZCur = 0;

    imuShiftFromStartXCur = 0;
    imuShiftFromStartYCur = 0;
    imuShiftFromStartZCur = 0;
    imuVeloFromStartXCur  = 0;
    imuVeloFromStartYCur  = 0;
    imuVeloFromStartZCur  = 0;

    imuAngularRotationXCur  = 0;
    imuAngularRotationYCur  = 0;
    imuAngularRotationZCur  = 0;
    imuAngularRotationXLast = 0;
    imuAngularRotationYLast = 0;
    imuAngularRotationZLast = 0;
    imuAngularFromStartX    = 0;
    imuAngularFromStartY    = 0;
    imuAngularFromStartZ    = 0;

    for (int i = 0; i < imuQueLength; ++i)
    {
      imuTime[i]             = 0;
      imuRoll[i]             = 0;
      imuPitch[i]            = 0;
      imuYaw[i]              = 0;
      imuAccX[i]             = 0;
      imuAccY[i]             = 0;
      imuAccZ[i]             = 0;
      imuVeloX[i]            = 0;
      imuVeloY[i]            = 0;
      imuVeloZ[i]            = 0;
      imuShiftX[i]           = 0;
      imuShiftY[i]           = 0;
      imuShiftZ[i]           = 0;
      imuAngularVeloX[i]     = 0;
      imuAngularVeloY[i]     = 0;
      imuAngularVeloZ[i]     = 0;
      imuAngularRotationX[i] = 0;
      imuAngularRotationY[i] = 0;
      imuAngularRotationZ[i] = 0;
    }

    skipFrameNum = 1;

    for (int i = 0; i < 6; ++i)
    {
      transformCur[i] = 0;
      transformSum[i] = 0;
    }

    systemInitedLM = false;

    imuRollLast        = 0;
    imuPitchLast       = 0;
    imuYawLast         = 0;
    imuShiftFromStartX = 0;
    imuShiftFromStartY = 0;
    imuShiftFromStartZ = 0;
    imuVeloFromStartX  = 0;
    imuVeloFromStartY  = 0;
    imuVeloFromStartZ  = 0;

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());

    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id  = "/laser_odom";

    laserOdometryTrans.frame_id_       = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom";

    isDegenerate = false;
    matP         = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    frameCount = skipFrameNum;
  }

  void updateImuRollPitchYawStartSinCos()
  {
    cosImuRollStart  = cos(imuRollStart);
    cosImuPitchStart = cos(imuPitchStart);
    cosImuYawStart   = cos(imuYawStart);
    sinImuRollStart  = sin(imuRollStart);
    sinImuPitchStart = sin(imuPitchStart);
    sinImuYawStart   = sin(imuYawStart);
  }

  //???
  void ShiftToStartIMU(float pointTime)
  {
    imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
    imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
    imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

    float x1 = cosImuYawStart * imuShiftFromStartXCur - sinImuYawStart * imuShiftFromStartZCur;
    float y1 = imuShiftFromStartYCur;
    float z1 = sinImuYawStart * imuShiftFromStartXCur + cosImuYawStart * imuShiftFromStartZCur;

    float x2 = x1;
    float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
    float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

    imuShiftFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
    imuShiftFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
    imuShiftFromStartZCur = z2;
  }
  //???
  void VeloToStartIMU()
  {
    imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
    imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
    imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

    float x1 = cosImuYawStart * imuVeloFromStartXCur - sinImuYawStart * imuVeloFromStartZCur;
    float y1 = imuVeloFromStartYCur;
    float z1 = sinImuYawStart * imuVeloFromStartXCur + cosImuYawStart * imuVeloFromStartZCur;

    float x2 = x1;
    float y2 = cosImuPitchStart * y1 + sinImuPitchStart * z1;
    float z2 = -sinImuPitchStart * y1 + cosImuPitchStart * z1;

    imuVeloFromStartXCur = cosImuRollStart * x2 + sinImuRollStart * y2;
    imuVeloFromStartYCur = -sinImuRollStart * x2 + cosImuRollStart * y2;
    imuVeloFromStartZCur = z2;
  }
  //???
  void TransformToStartIMU(PointType *p)
  {
    float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
    float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    float z1 = p->z;

    float x2 = x1;
    float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
    float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
    float y3 = y2;
    float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    float x4 = cosImuYawStart * x3 - sinImuYawStart * z3;
    float y4 = y3;
    float z4 = sinImuYawStart * x3 + cosImuYawStart * z3;

    float x5 = x4;
    float y5 = cosImuPitchStart * y4 + sinImuPitchStart * z4;
    float z5 = -sinImuPitchStart * y4 + cosImuPitchStart * z4;

    p->x = cosImuRollStart * x5 + sinImuRollStart * y5 + imuShiftFromStartXCur;
    p->y = -sinImuRollStart * x5 + cosImuRollStart * y5 + imuShiftFromStartYCur;
    p->z = z5 + imuShiftFromStartZCur;
  }
  // 积分IMU的位移和旋转
  void AccumulateIMUShiftAndRotation()
  {
    float roll  = imuRoll[imuPointerLast];
    float pitch = imuPitch[imuPointerLast];
    float yaw   = imuYaw[imuPointerLast];
    float accX  = imuAccX[imuPointerLast];
    float accY  = imuAccY[imuPointerLast];
    float accZ  = imuAccZ[imuPointerLast];

    // 坐标系转换
    float x1 = cos(roll) * accX - sin(roll) * accY;
    float y1 = sin(roll) * accX + cos(roll) * accY;
    float z1 = accZ;

    float x2 = x1;
    float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    float z2 = sin(pitch) * y1 + cos(pitch) * z1;

    accX = cos(yaw) * x2 + sin(yaw) * z2;
    accY = y2;
    accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
    double timeDiff    = imuTime[imuPointerLast] - imuTime[imuPointerBack];
    if (timeDiff < scanPeriod)
    { // x = x0 + v*△t + 0.5 a*t^2
      imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
      imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
      imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

      imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
      imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
      imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;

      imuAngularRotationX[imuPointerLast] = imuAngularRotationX[imuPointerBack] + imuAngularVeloX[imuPointerBack] * timeDiff;
      imuAngularRotationY[imuPointerLast] = imuAngularRotationY[imuPointerBack] + imuAngularVeloY[imuPointerBack] * timeDiff;
      imuAngularRotationZ[imuPointerLast] = imuAngularRotationZ[imuPointerBack] + imuAngularVeloZ[imuPointerBack] * timeDiff;
    }
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn)
  {
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // 把IMU坐标系的加速度值转换到世界坐标系下
    float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
    float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
    float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

    imuPointerLast = (imuPointerLast + 1) % imuQueLength;

    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();

    imuRoll[imuPointerLast]  = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast]   = yaw;

    imuAccX[imuPointerLast] = accX;
    imuAccY[imuPointerLast] = accY;
    imuAccZ[imuPointerLast] = accZ;

    imuAngularVeloX[imuPointerLast] = imuIn->angular_velocity.x;
    imuAngularVeloY[imuPointerLast] = imuIn->angular_velocity.y;
    imuAngularVeloZ[imuPointerLast] = imuIn->angular_velocity.z;

    AccumulateIMUShiftAndRotation();
  }

  void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    cloudHeader = laserCloudMsg->header;

    timeScanCur           = cloudHeader.stamp.toSec();
    timeNewSegmentedCloud = timeScanCur;

    segmentedCloud->clear();
    pcl::fromROSMsg(*laserCloudMsg, *segmentedCloud);

    newSegmentedCloud = true;
  }

  void outlierCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msgIn)
  {
    timeNewOutlierCloud = msgIn->header.stamp.toSec();

    outlierCloud->clear();
    pcl::fromROSMsg(*msgIn, *outlierCloud);

    newOutlierCloud = true;
  }

  void laserCloudInfoHandler(const cloud_msgs::cloud_infoConstPtr &msgIn)
  {
    timeNewSegmentedCloudInfo = msgIn->header.stamp.toSec();
    segInfo                   = *msgIn;
    newSegmentedCloudInfo     = true;
  }

  // ???
  void adjustDistortion()
  {
    bool halfPassed = false; // 判断剩下的点是不是属于后一半的点，用于计算角度
    int cloudSize   = segmentedCloud->points.size();

    PointType point;

    for (int i = 0; i < cloudSize; i++)
    {
      point.x = segmentedCloud->points[i].y;
      point.y = segmentedCloud->points[i].z;
      point.z = segmentedCloud->points[i].x;

      float ori = -atan2(point.x, point.z); // 范围为(-90,90)
      // 结合该点是否过一半，把角度从(-90,90)转到(0,360)
      if (!halfPassed)
      {
        if (ori < segInfo.startOrientation - M_PI / 2)
        {
          ori += 2 * M_PI;
        }
        else if (ori > segInfo.startOrientation + M_PI * 3 / 2)
        {
          ori -= 2 * M_PI;
        }

        if (ori - segInfo.startOrientation > M_PI)
        {
          halfPassed = true;
        }
      }
      else
      {
        ori += 2 * M_PI;

        if (ori < segInfo.endOrientation - M_PI * 3 / 2)
        {
          ori += 2 * M_PI;
        }
        else if (ori > segInfo.endOrientation + M_PI / 2)
        {
          ori -= 2 * M_PI;
        }
      }

      // 匀速旋转假设，根据角度计算该点的时间
      float relTime = (ori - segInfo.startOrientation) / segInfo.orientationDiff;
      // ???整数部分为行号；小数部分为该点到点云起始时刻的时间
      point.intensity = int(segmentedCloud->points[i].intensity) + scanPeriod * relTime;

      if (imuPointerLast >= 0)
      {
        float pointTime = relTime * scanPeriod;
        imuPointerFront = imuPointerLastIteration;
        while (imuPointerFront != imuPointerLast) // 如果还有IMU数据就可以继续循环
        {
          if (timeScanCur + pointTime < imuTime[imuPointerFront])
          {
            break;
          }
          imuPointerFront = (imuPointerFront + 1) % imuQueLength;
        }

        if (timeScanCur + pointTime > imuTime[imuPointerFront])
        { // 所有IMU数据都比这个点的时间早
          imuRollCur  = imuRoll[imuPointerFront];
          imuPitchCur = imuPitch[imuPointerFront];
          imuYawCur   = imuYaw[imuPointerFront];

          imuVeloXCur = imuVeloX[imuPointerFront];
          imuVeloYCur = imuVeloY[imuPointerFront];
          imuVeloZCur = imuVeloZ[imuPointerFront];

          imuShiftXCur = imuShiftX[imuPointerFront];
          imuShiftYCur = imuShiftY[imuPointerFront];
          imuShiftZCur = imuShiftZ[imuPointerFront];
        }
        else
        { // 存在比该点早和比该点晚的IMU数据
          int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
          float ratioFront   = (timeScanCur + pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
          float ratioBack    = (imuTime[imuPointerFront] - timeScanCur - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
          // 线性插值计算角度
          imuRollCur  = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
          imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
          if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI)
          {
            imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
          }
          else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI)
          {
            imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
          }
          else
          {
            imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
          }
          // 线性插值计算速度
          imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
          imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
          imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;
          // 线性插值计算位移
          imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
          imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
          imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
        }

        if (i == 0)
        { // 这帧点云的第一个点
          imuRollStart  = imuRollCur;
          imuPitchStart = imuPitchCur;
          imuYawStart   = imuYawCur;

          imuVeloXStart = imuVeloXCur;
          imuVeloYStart = imuVeloYCur;
          imuVeloZStart = imuVeloZCur;

          imuShiftXStart = imuShiftXCur;
          imuShiftYStart = imuShiftYCur;
          imuShiftZStart = imuShiftZCur;

          if (timeScanCur + pointTime > imuTime[imuPointerFront])
          { // 结合前面的while循环可知，此时IMU的所有数据都早于第一个点的时间
            imuAngularRotationXCur = imuAngularRotationX[imuPointerFront];
            imuAngularRotationYCur = imuAngularRotationY[imuPointerFront];
            imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront];
          }
          else
          { // 线性插值
            int imuPointerBack     = (imuPointerFront + imuQueLength - 1) % imuQueLength;
            float ratioFront       = (timeScanCur + pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            float ratioBack        = (imuTime[imuPointerFront] - timeScanCur - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            imuAngularRotationXCur = imuAngularRotationX[imuPointerFront] * ratioFront + imuAngularRotationX[imuPointerBack] * ratioBack;
            imuAngularRotationYCur = imuAngularRotationY[imuPointerFront] * ratioFront + imuAngularRotationY[imuPointerBack] * ratioBack;
            imuAngularRotationZCur = imuAngularRotationZ[imuPointerFront] * ratioFront + imuAngularRotationZ[imuPointerBack] * ratioBack;
          }

          imuAngularFromStartX = imuAngularRotationXCur - imuAngularRotationXLast;
          imuAngularFromStartY = imuAngularRotationYCur - imuAngularRotationYLast;
          imuAngularFromStartZ = imuAngularRotationZCur - imuAngularRotationZLast;

          imuAngularRotationXLast = imuAngularRotationXCur;
          imuAngularRotationYLast = imuAngularRotationYCur;
          imuAngularRotationZLast = imuAngularRotationZCur;

          updateImuRollPitchYawStartSinCos();
        } // 处理完第一个点
        else
        {
          VeloToStartIMU();
          TransformToStartIMU(&point);
        }
      } //
      segmentedCloud->points[i] = point;
    } // endfor: have calculated all points' pose and velocity

    imuPointerLastIteration = imuPointerLast;
  }

  void calculateSmoothness()
  {
    int cloudSize = segmentedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    { // 前五个点的距离属性之和加后五个点的距离之和-10倍该点的距离,算出差值，确定连续点之间的起伏趋势
      float diffRange = segInfo.segmentedCloudRange[i - 5] + segInfo.segmentedCloudRange[i - 4] + segInfo.segmentedCloudRange[i - 3] + segInfo.segmentedCloudRange[i - 2] + segInfo.segmentedCloudRange[i - 1] - segInfo.segmentedCloudRange[i] * 10 + segInfo.segmentedCloudRange[i + 1] + segInfo.segmentedCloudRange[i + 2] + segInfo.segmentedCloudRange[i + 3] + segInfo.segmentedCloudRange[i + 4] + segInfo.segmentedCloudRange[i + 5];

      cloudCurvature[i] = diffRange * diffRange;

      cloudNeighborPicked[i] = 0;
      cloudLabel[i]          = 0;
      // 记录曲率以及对应点的索引, 后续需要根据曲率进行排序，打乱点的顺序, 只能通过ind找到点
      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind   = i;
    }
  }
  // according to the article, there are several kinds of points ,from which we should't extract features
  void markOccludedPoints()
  {
    int cloudSize = segmentedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i)
    {
      float depth1 = segInfo.segmentedCloudRange[i];
      float depth2 = segInfo.segmentedCloudRange[i + 1];
      // 深度图中列的差值
      int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i + 1] - segInfo.segmentedCloudColInd[i]));

      // 平行线和遮挡的判断参考LOAM
      if (columnDiff < 10)
      {
        // difference of column is small and difference of range is large
        if (depth1 - depth2 > 0.3)
        {
          cloudNeighborPicked[i - 5] = 1; // marked as "1", will not be processed
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i]     = 1;
        }
        else if (depth2 - depth1 > 0.3)
        {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
      // parallel beam; 平行线的情况,根据左右两点与该点的深度差,确定该点是否会被选择为特征点
      float diff1 = std::abs(float(segInfo.segmentedCloudRange[i - 1] - segInfo.segmentedCloudRange[i]));
      float diff2 = std::abs(float(segInfo.segmentedCloudRange[i + 1] - segInfo.segmentedCloudRange[i]));

      if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
      {
        cloudNeighborPicked[i] = 1;
      }
    }
  }

  void extractFeatures()
  {
    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();

    for (int i = 0; i < N_SCAN; i++)
    {
      surfPointsLessFlatScan->clear();

      // 为了保证各方向均匀提取, 将深度图分为6个子图,
      for (int j = 0; j < 6; j++)
      {
        // start and end point's index
        int sp = (segInfo.startRingIndex[i] * (6 - j) + segInfo.endRingIndex[i] * j) / 6;
        int ep = (segInfo.startRingIndex[i] * (5 - j) + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

        if (sp >= ep)
        {
          continue;
        }
        // 每个子图中对点的曲率进行排序，sp和ep分别是这段点云的起始点与终止点
        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

        int largestPickedNum = 0;
        // 标记平面点: find sharp/edge in one single segment
        for (int k = ep; k >= sp; k--)
        {
          int ind = cloudSmoothness[k].ind;
          // 曲率比较大的是边缘点
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold && segInfo.segmentedCloudGroundFlag[ind] == false)
          {
            largestPickedNum++;
            if (largestPickedNum <= 2)
            {
              cloudLabel[ind] = 2;
              cornerPointsSharp->push_back(segmentedCloud->points[ind]);
              cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
            }
            else if (largestPickedNum <= 20) // 选取20个边缘点
            {
              cloudLabel[ind] = 1; // edge feature
              cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
            }
            else
            {
              break;
            }

            cloudNeighborPicked[ind] = 1;
            // 防止特征点聚集: shouldn't ectract feature point from neighbor area of a feature point
            for (int l = 1; l <= 5; l++)
            {
              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
              if (columnDiff > 10)
              {
                break; // means not neighbor area
              }
              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        } // endfor:

        int smallestPickedNum = 0;
        // 标记平面点
        for (int k = sp; k <= ep; k++)
        {
          int ind = cloudSmoothness[k].ind;
          // 在还没有标记并且曲率较小的点里面选
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold && segInfo.segmentedCloudGroundFlag[ind] == true)
          {
            cloudLabel[ind] = -1; // surf feature
            surfPointsFlat->push_back(segmentedCloud->points[ind]);

            smallestPickedNum++;
            if (smallestPickedNum >= 4)
            {
              break;
            }

            cloudNeighborPicked[ind] = 1;
            // 防止平面点聚集
            for (int l = 1; l <= 5; l++)
            {
              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
              if (columnDiff > 10)
                break;

              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
              if (columnDiff > 10)
                break;

              cloudNeighborPicked[ind + l] = 1;
            }
          }
        } // endfor: 已经标记平面点
          // label 小于 0 的点是平面点
        for (int k = sp; k <= ep; k++)
        {
          if (cloudLabel[k] <= 0)
          { // flat and less flat
            // 保存面点
            surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
          }
        }
      } // endfor:已经标记了所有角点和面点

      // 下采样每行的面点
      surfPointsLessFlatScanDS->clear();
      downSizeFilter.setInputCloud(surfPointsLessFlatScan);
      downSizeFilter.filter(*surfPointsLessFlatScanDS);
      // 保存下采样后的面点
      *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    } // endfor: 已经遍历了每一行的点
  }

  void publishCloud()
  {
    sensor_msgs::PointCloud2 laserCloudOutMsg;

    if (pubCornerPointsSharp.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*cornerPointsSharp, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp    = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "/camera";
      pubCornerPointsSharp.publish(laserCloudOutMsg);
    }

    if (pubCornerPointsLessSharp.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*cornerPointsLessSharp, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp    = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "/camera";
      pubCornerPointsLessSharp.publish(laserCloudOutMsg);
    }

    if (pubSurfPointsFlat.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*surfPointsFlat, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp    = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "/camera";
      pubSurfPointsFlat.publish(laserCloudOutMsg);
    }

    if (pubSurfPointsLessFlat.getNumSubscribers() != 0)
    {
      pcl::toROSMsg(*surfPointsLessFlat, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp    = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "/camera";
      pubSurfPointsLessFlat.publish(laserCloudOutMsg);
    }
  }

  void TransformToStart(PointType const *const pi, PointType *const po)
  {
    float s = 10 * (pi->intensity - int(pi->intensity)); // 10是雷达的频率

    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    po->x         = cos(ry) * x2 - sin(ry) * z2;
    po->y         = y2;
    po->z         = sin(ry) * x2 + cos(ry) * z2;
    po->intensity = pi->intensity;
  }

  void TransformToEnd(PointType const *const pi, PointType *const po)
  {
    float s = 10 * (pi->intensity - int(pi->intensity));

    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    float x3 = cos(ry) * x2 - sin(ry) * z2;
    float y3 = y2;
    float z3 = sin(ry) * x2 + cos(ry) * z2;

    rx = transformCur[0];
    ry = transformCur[1];
    rz = transformCur[2];
    tx = transformCur[3];
    ty = transformCur[4];
    tz = transformCur[5];

    float x4 = cos(ry) * x3 + sin(ry) * z3;
    float y4 = y3;
    float z4 = -sin(ry) * x3 + cos(ry) * z3;

    float x5 = x4;
    float y5 = cos(rx) * y4 - sin(rx) * z4;
    float z5 = sin(rx) * y4 + cos(rx) * z4;

    float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
    float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
    float z6 = z5 + tz;

    float x7 = cosImuRollStart * (x6 - imuShiftFromStartX) - sinImuRollStart * (y6 - imuShiftFromStartY);
    float y7 = sinImuRollStart * (x6 - imuShiftFromStartX) + cosImuRollStart * (y6 - imuShiftFromStartY);
    float z7 = z6 - imuShiftFromStartZ;

    float x8 = x7;
    float y8 = cosImuPitchStart * y7 - sinImuPitchStart * z7;
    float z8 = sinImuPitchStart * y7 + cosImuPitchStart * z7;

    float x9 = cosImuYawStart * x8 + sinImuYawStart * z8;
    float y9 = y8;
    float z9 = -sinImuYawStart * x8 + cosImuYawStart * z8;

    float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
    float y10 = y9;
    float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

    float x11 = x10;
    float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
    float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

    po->x         = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
    po->y         = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
    po->z         = z11;
    po->intensity = int(pi->intensity);
  }

  void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz,
                         float alx, float aly, float alz, float &acx, float &acy, float &acz)
  {
    float sbcx = sin(bcx);
    float cbcx = cos(bcx);
    float sbcy = sin(bcy);
    float cbcy = cos(bcy);
    float sbcz = sin(bcz);
    float cbcz = cos(bcz);

    float sblx = sin(blx);
    float cblx = cos(blx);
    float sbly = sin(bly);
    float cbly = cos(bly);
    float sblz = sin(blz);
    float cblz = cos(blz);

    float salx = sin(alx);
    float calx = cos(alx);
    float saly = sin(aly);
    float caly = cos(aly);
    float salz = sin(alz);
    float calz = cos(alz);

    float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly) - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
    acx       = -asin(srx);

    float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
    float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz) - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * saly * (cbly * sblz - cblz * sblx * sbly) - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx) + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
    acy          = atan2(srycrx / cos(acx), crycrx / cos(acx));

    float srzcrx = sbcx * (cblx * cbly * (calz * saly - caly * salx * salz) - cblx * sbly * (caly * calz + salx * saly * salz) + calx * salz * sblx) - cbcx * cbcz * ((caly * calz + salx * saly * salz) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cblz * salz) + cbcx * sbcz * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * cblx * salz * sblz);
    float crzcrx = sbcx * (cblx * sbly * (caly * salz - calz * salx * saly) - cblx * cbly * (saly * salz + caly * calz * salx) + calx * calz * sblx) + cbcx * cbcz * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * calz * cblx * cblz) - cbcx * sbcz * ((saly * salz + caly * calz * salx) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (cbly * cblz + sblx * sbly * sblz) - calx * calz * cblx * sblz);
    acz          = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
  }

  void AccumulateRotation(float cx, float cy, float cz,
                          float lx, float ly, float lz,
                          float &ox, float &oy, float &oz)
  {
    float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) - cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
    ox        = -asin(srx);

    float srycrx = sin(lx) * (cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) + cos(lx) * sin(ly) * (cos(cy) * cos(cz) + sin(cx) * sin(cy) * sin(cz)) + cos(lx) * cos(ly) * cos(cx) * sin(cy);
    float crycrx = cos(lx) * cos(ly) * cos(cx) * cos(cy) - cos(lx) * sin(ly) * (cos(cz) * sin(cy) - cos(cy) * sin(cx) * sin(cz)) - sin(lx) * (sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
    oy           = atan2(srycrx / cos(ox), crycrx / cos(ox));

    float srzcrx = sin(cx) * (cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) + cos(cx) * sin(cz) * (cos(ly) * cos(lz) + sin(lx) * sin(ly) * sin(lz)) + cos(lx) * cos(cx) * cos(cz) * sin(lz);
    float crzcrx = cos(lx) * cos(lz) * cos(cx) * cos(cz) - cos(cx) * sin(cz) * (cos(ly) * sin(lz) - cos(lz) * sin(lx) * sin(ly)) - sin(cx) * (sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
    oz           = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
  }

  double rad2deg(double radians)
  {
    return radians * 180.0 / M_PI;
  }

  double deg2rad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  void findCorrespondingCornerFeatures(int iterCount)
  {
    int cornerPointsSharpNum = cornerPointsSharp->points.size();

    for (int i = 0; i < cornerPointsSharpNum; i++)
    {
      TransformToStart(&cornerPointsSharp->points[i], &pointSel);

      if (iterCount % 5 == 0)
      {
        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
        int closestPointInd = -1, minPointInd2 = -1;

        if (pointSearchSqDis[0] < nearestFeatureSearchSqDist)
        {
          closestPointInd      = pointSearchInd[0];
          int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

          float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
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
          } // endfor:
        }

        pointSearchCornerInd1[i] = closestPointInd;
        pointSearchCornerInd2[i] = minPointInd2;
      }

      if (pointSearchCornerInd2[i] >= 0)
      {
        tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
        tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

        float x0 = pointSel.x;
        float y0 = pointSel.y;
        float z0 = pointSel.z;
        float x1 = tripod1.x;
        float y1 = tripod1.y;
        float z1 = tripod1.z;
        float x2 = tripod2.x;
        float y2 = tripod2.y;
        float z2 = tripod2.z;

        float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
        float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
        float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

        float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

        float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;

        float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;

        float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;

        float ld2 = a012 / l12;

        float s = 1;
        if (iterCount >= 5)
        {
          s = 1 - 1.8 * fabs(ld2);
        }

        if (s > 0.1 && ld2 != 0)
        {
          coeff.x         = s * la;
          coeff.y         = s * lb;
          coeff.z         = s * lc;
          coeff.intensity = s * ld2;

          laserCloudOri->push_back(cornerPointsSharp->points[i]);
          coeffSel->push_back(coeff);
        }
      }
    } // endfor:
  }

  void findCorrespondingSurfFeatures(int iterCount)
  {
    int surfPointsFlatNum = surfPointsFlat->points.size();
    // 为每一个面点寻找对应的平面特征
    for (int i = 0; i < surfPointsFlatNum; i++)
    {
      TransformToStart(&surfPointsFlat->points[i], &pointSel);

      // 每次迭代后，最近点的结果都可能会不同，如果沿用之前的匹配结果，会影响精度;
      // 为平衡计算量和精度，每迭代5次才重新查找最近点
      if (iterCount % 5 == 0)
      {
        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
        // 根据论文，需要三个最近点组成面特征，分别为： 最近点；同一行的另一最近点；相邻行的另一最近点
        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        // 5个最近点中，最近的距离不得超过1米，否则为无效约束
        if (pointSearchSqDis[0] < nearestFeatureSearchSqDist)
        {
          closestPointInd      = pointSearchInd[0];                                          // 最近点的索引
          int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity); // 最近点所在的行
          // 更新最近点
          float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
          // 按索引增加方向搜索另一个最近点
          for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
          {
            // 只在较近的行里搜索
            if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5)
            {
              break;
            }

            pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x)
                         + (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y)
                         + (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);

            if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan)
            { // 同一行
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2   = j;
              }
            }
            else
            { // 相邻行
              if (pointSqDis < minPointSqDis3)
              {
                minPointSqDis3 = pointSqDis;
                minPointInd3   = j;
              }
            }
          } // endfor: 完成索引增加方向的搜索

          // 按索引减小方向搜索另一个最近点
          for (int j = closestPointInd - 1; j >= 0; j--)
          {
            // 只在较近的行里搜索
            if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5)
            {
              break;
            }

            pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x)
                         + (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y)
                         + (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);

            if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan)
            { // 同一行
              if (pointSqDis < minPointSqDis2)
              {
                minPointSqDis2 = pointSqDis;
                minPointInd2   = j;
              }
            }
            else
            { // 相邻行
              if (pointSqDis < minPointSqDis3)
              {
                minPointSqDis3 = pointSqDis;
                minPointInd3   = j;
              }
            }
          } // endfor: 完成索引减小方向的搜索

        } // 如果最近点有效，查找其它最近点组成面特征

        pointSearchSurfInd1[i] = closestPointInd;
        pointSearchSurfInd2[i] = minPointInd2;
        pointSearchSurfInd3[i] = minPointInd3;
      } // 完成查找并更新最近点

      // 如果存在有效的最近点
      if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0)
      {
        tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
        tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
        tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];
        // 法向量参数 ax+by+cz +d = 0
        float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
        float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
        float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
        float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

        float ps = sqrt(pa * pa + pb * pb + pc * pc);

        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;
        // residual: distance between point and plane
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1;
        if (iterCount >= 5)
        {
          s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
        }

        if (s > 0.1 && pd2 != 0)
        {
          coeff.x         = s * pa;
          coeff.y         = s * pb;
          coeff.z         = s * pc;
          coeff.intensity = s * pd2;

          laserCloudOri->push_back(surfPointsFlat->points[i]);
          coeffSel->push_back(coeff); // jacobi and residual
        }
      } // endif: 存在有效的最近点并更新对应的平面特征

    } // endfor: 为每一个面点寻找对应的平面特征
  }

  bool calculateTransformationSurf(int iterCount)
  {
    int pointSelNum = laserCloudOri->points.size();
    // 高斯牛顿优化
    // 每个点的残差的旋转和平移的雅可比;旋转的雅可比需要求导,平移的雅可比为残差的方向
    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0)); // A.transpose()
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));          // A.transpose() * A
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));  // 残差的大小
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));          // A.transpose() * B
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));            // 步进的大小和方向

    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx  = transformCur[3];
    float ty  = transformCur[4];
    float tz  = transformCur[5];

    float a1  = crx * sry * srz;
    float a2  = crx * crz * sry;
    float a3  = srx * sry;
    float a4  = tx * a1 - ty * a2 - tz * a3;
    float a5  = srx * srz;
    float a6  = crz * srx;
    float a7  = ty * a6 - tz * crx - tx * a5;
    float a8  = crx * cry * srz;
    float a9  = crx * cry * crz;
    float a10 = cry * srx;
    float a11 = tz * a10 + ty * a9 - tx * a8;

    float b1 = -crz * sry - cry * srx * srz;
    float b2 = cry * crz * srx - sry * srz;
    float b5 = cry * crz - srx * sry * srz;
    float b6 = cry * srz + crz * srx * sry;

    float c1 = -b6;
    float c2 = b5;
    float c3 = tx * b6 - ty * b5;
    float c4 = -crx * crz;
    float c5 = crx * srz;
    float c6 = ty * c5 + tx * -c4;
    float c7 = b2;
    float c8 = -b1;
    float c9 = tx * -b2 - ty * -b1;

    for (int i = 0; i < pointSelNum; i++)
    {
      pointOri = laserCloudOri->points[i];
      coeff    = coeffSel->points[i];
      // calculate jacobi of rotation
      float arx = (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x
                  + (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y
                  + (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

      float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x
                  + (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y
                  + (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

      float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

      float d2 = coeff.intensity;

      matA.at<float>(i, 0) = arx; // jacobi of rotation
      matA.at<float>(i, 1) = arz;
      matA.at<float>(i, 2) = aty;
      matB.at<float>(i, 0) = -0.05 * d2; // residual
    }
    // JTJ * deltax = -JTe (J:Jacobi of e; e:residual, scalar quantity)
    // construct JTJ and -JTe matrix
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    // QR factorization; the system can be over-defined and/or the matrix src1 can be singular
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR); // Ax=B->get x(deltax)

    if (iterCount == 0)
    { // check whether degenerated
      cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate      = false;
      float eignThre[3] = {10, 10, 10};
      for (int i = 2; i >= 0; i--)
      {
        // eigenvalue < eigenthresold,特征值太小,说明矩阵在相应的特征向量方向退化
        if (matE.at<float>(0, i) < eignThre[i])
        {
          for (int j = 0; j < 3; j++)
          {
            matV2.at<float>(i, j) = 0; // corresponding vector is set as zero
          }
          isDegenerate = true; // there is degenarated component
        }
        else
        {
          break;
        }
      }
      matP = matV.inv() * matV2; // eigenvector matrix without degenerated component
    }
    // for example, in a long corridor, cant estimate translation in direction of corridor,
    // thus, corresponding vector is degenerated and corresponding component of delta x should be zero
    if (isDegenerate)
    { // set degenerated component of delta x as zero
      cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformCur[0] += matX.at<float>(0, 0);
    transformCur[2] += matX.at<float>(1, 0);
    transformCur[4] += matX.at<float>(2, 0);

    for (int i = 0; i < 6; i++)
    {
      if (isnan(transformCur[i]))
      {
        transformCur[i] = 0;
      }
    }
    // unit is degree
    float deltaR = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) + pow(rad2deg(matX.at<float>(1, 0)), 2));
    // unit is cm
    float deltaT = sqrt(pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1)
    {
      return false; // pose has converged, end iteration
    }
    return true; // pose hasn't converged, keep iteration
  }

  bool calculateTransformationCorner(int iterCount)
  {
    int pointSelNum = laserCloudOri->points.size();

    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx  = transformCur[3];
    float ty  = transformCur[4];
    float tz  = transformCur[5];

    float b1 = -crz * sry - cry * srx * srz;
    float b2 = cry * crz * srx - sry * srz;
    float b3 = crx * cry;
    float b4 = tx * -b1 + ty * -b2 + tz * b3;
    float b5 = cry * crz - srx * sry * srz;
    float b6 = cry * srz + crz * srx * sry;
    float b7 = crx * sry;
    float b8 = tz * b7 - ty * b6 - tx * b5;

    float c5 = crx * srz;

    for (int i = 0; i < pointSelNum; i++)
    {
      pointOri = laserCloudOri->points[i];
      coeff    = coeffSel->points[i];

      float ary = (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x + (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

      float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

      float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

      float d2 = coeff.intensity;

      matA.at<float>(i, 0) = ary;
      matA.at<float>(i, 1) = atx;
      matA.at<float>(i, 2) = atz;
      matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0)
    {
      cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate      = false;
      float eignThre[3] = {10, 10, 10};
      for (int i = 2; i >= 0; i--)
      {
        if (matE.at<float>(0, i) < eignThre[i])
        {
          for (int j = 0; j < 3; j++)
          {
            matV2.at<float>(i, j) = 0;
          }
          isDegenerate = true;
        }
        else
        {
          break;
        }
      }
      matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
      cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformCur[1] += matX.at<float>(0, 0);
    transformCur[3] += matX.at<float>(1, 0);
    transformCur[5] += matX.at<float>(2, 0);

    for (int i = 0; i < 6; i++)
    {
      if (isnan(transformCur[i]))
        transformCur[i] = 0;
    }

    float deltaR = sqrt(
        pow(rad2deg(matX.at<float>(0, 0)), 2));
    float deltaT = sqrt(
        pow(matX.at<float>(1, 0) * 100, 2) + pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1)
    {
      return false;
    }
    return true;
  }

  bool calculateTransformation(int iterCount)
  {
    int pointSelNum = laserCloudOri->points.size();

    cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx  = transformCur[3];
    float ty  = transformCur[4];
    float tz  = transformCur[5];

    float a1  = crx * sry * srz;
    float a2  = crx * crz * sry;
    float a3  = srx * sry;
    float a4  = tx * a1 - ty * a2 - tz * a3;
    float a5  = srx * srz;
    float a6  = crz * srx;
    float a7  = ty * a6 - tz * crx - tx * a5;
    float a8  = crx * cry * srz;
    float a9  = crx * cry * crz;
    float a10 = cry * srx;
    float a11 = tz * a10 + ty * a9 - tx * a8;

    float b1 = -crz * sry - cry * srx * srz;
    float b2 = cry * crz * srx - sry * srz;
    float b3 = crx * cry;
    float b4 = tx * -b1 + ty * -b2 + tz * b3;
    float b5 = cry * crz - srx * sry * srz;
    float b6 = cry * srz + crz * srx * sry;
    float b7 = crx * sry;
    float b8 = tz * b7 - ty * b6 - tx * b5;

    float c1 = -b6;
    float c2 = b5;
    float c3 = tx * b6 - ty * b5;
    float c4 = -crx * crz;
    float c5 = crx * srz;
    float c6 = ty * c5 + tx * -c4;
    float c7 = b2;
    float c8 = -b1;
    float c9 = tx * -b2 - ty * -b1;

    for (int i = 0; i < pointSelNum; i++)
    {
      pointOri = laserCloudOri->points[i];
      coeff    = coeffSel->points[i];

      float arx = (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x + (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y + (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

      float ary = (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x + (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

      float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x + (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y + (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

      float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

      float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

      float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

      float d2 = coeff.intensity;

      matA.at<float>(i, 0) = arx;
      matA.at<float>(i, 1) = ary;
      matA.at<float>(i, 2) = arz;
      matA.at<float>(i, 3) = atx;
      matA.at<float>(i, 4) = aty;
      matA.at<float>(i, 5) = atz;
      matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0)
    {
      cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate      = false;
      float eignThre[6] = {10, 10, 10, 10, 10, 10};
      for (int i = 5; i >= 0; i--)
      {
        if (matE.at<float>(0, i) < eignThre[i])
        {
          for (int j = 0; j < 6; j++)
          {
            matV2.at<float>(i, j) = 0;
          }
          isDegenerate = true;
        }
        else
        {
          break;
        }
      }
      matP = matV.inv() * matV2;
    }

    if (isDegenerate)
    {
      cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformCur[0] += matX.at<float>(0, 0);
    transformCur[1] += matX.at<float>(1, 0);
    transformCur[2] += matX.at<float>(2, 0);
    transformCur[3] += matX.at<float>(3, 0);
    transformCur[4] += matX.at<float>(4, 0);
    transformCur[5] += matX.at<float>(5, 0);

    for (int i = 0; i < 6; i++)
    {
      if (isnan(transformCur[i]))
        transformCur[i] = 0;
    }

    float deltaR = sqrt(
        pow(rad2deg(matX.at<float>(0, 0)), 2) + pow(rad2deg(matX.at<float>(1, 0)), 2) + pow(rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
        pow(matX.at<float>(3, 0) * 100, 2) + pow(matX.at<float>(4, 0) * 100, 2) + pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1)
    {
      return false;
    }
    return true;
  }

  void checkSystemInitialization()
  {
    pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp                          = laserCloudCornerLast;
    laserCloudCornerLast                           = laserCloudTemp;

    laserCloudTemp     = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum   = laserCloudSurfLast->points.size();

    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp    = cloudHeader.stamp;
    laserCloudCornerLast2.header.frame_id = "/camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp    = cloudHeader.stamp;
    laserCloudSurfLast2.header.frame_id = "/camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

    transformSum[0] += imuPitchStart;
    transformSum[2] += imuRollStart;

    systemInitedLM = true;
  }

  void updateInitialGuess()
  {
    imuPitchLast = imuPitchCur;
    imuYawLast   = imuYawCur;
    imuRollLast  = imuRollCur;

    imuShiftFromStartX = imuShiftFromStartXCur;
    imuShiftFromStartY = imuShiftFromStartYCur;
    imuShiftFromStartZ = imuShiftFromStartZCur;

    imuVeloFromStartX = imuVeloFromStartXCur;
    imuVeloFromStartY = imuVeloFromStartYCur;
    imuVeloFromStartZ = imuVeloFromStartZCur;

    if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0)
    {
      transformCur[0] = -imuAngularFromStartY;
      transformCur[1] = -imuAngularFromStartZ;
      transformCur[2] = -imuAngularFromStartX;
    }

    if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0)
    {
      transformCur[3] -= imuVeloFromStartX * scanPeriod;
      transformCur[4] -= imuVeloFromStartY * scanPeriod;
      transformCur[5] -= imuVeloFromStartZ * scanPeriod;
    }
  }

  void updateTransformation()
  {
    // there are enough feature points
    if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
    {
      return;
    }
    // iterative solution.
    for (int iterCount1 = 0; iterCount1 < 25; iterCount1++)
    {
      laserCloudOri->clear();
      coeffSel->clear();

      findCorrespondingSurfFeatures(iterCount1);

      if (laserCloudOri->points.size() < 10)
      {
        continue;
      }
      if (calculateTransformationSurf(iterCount1) == false)
      {
        break;
      }
    }
    //

    for (int iterCount2 = 0; iterCount2 < 25; iterCount2++)
    {
      laserCloudOri->clear();
      coeffSel->clear();

      findCorrespondingCornerFeatures(iterCount2);

      if (laserCloudOri->points.size() < 10)
      {
        continue;
      }
      if (calculateTransformationCorner(iterCount2) == false)
      {
        break;
      }
    }
    //
  }

  void integrateTransformation()
  {
    float rx, ry, rz, tx, ty, tz;
    AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                       -transformCur[0], -transformCur[1], -transformCur[2],
                       rx, ry, rz);

    float x1 = cos(rz) * (transformCur[3] - imuShiftFromStartX) - sin(rz) * (transformCur[4] - imuShiftFromStartY);
    float y1 = sin(rz) * (transformCur[3] - imuShiftFromStartX) + cos(rz) * (transformCur[4] - imuShiftFromStartY);
    float z1 = transformCur[5] - imuShiftFromStartZ;

    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
    ty = transformSum[4] - y2;
    tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

    PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart,
                      imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

    transformSum[0] = rx;
    transformSum[1] = ry;
    transformSum[2] = rz;
    transformSum[3] = tx;
    transformSum[4] = ty;
    transformSum[5] = tz;
  }

  void publishOdometry()
  {
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum[2], -transformSum[0], -transformSum[1]);

    laserOdometry.header.stamp            = cloudHeader.stamp;
    laserOdometry.pose.pose.orientation.x = -geoQuat.y;
    laserOdometry.pose.pose.orientation.y = -geoQuat.z;
    laserOdometry.pose.pose.orientation.z = geoQuat.x;
    laserOdometry.pose.pose.orientation.w = geoQuat.w;
    laserOdometry.pose.pose.position.x    = transformSum[3];
    laserOdometry.pose.pose.position.y    = transformSum[4];
    laserOdometry.pose.pose.position.z    = transformSum[5];
    pubLaserOdometry.publish(laserOdometry);

    laserOdometryTrans.stamp_ = cloudHeader.stamp;
    laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    laserOdometryTrans.setOrigin(tf::Vector3(transformSum[3], transformSum[4], transformSum[5]));
    tfBroadcaster.sendTransform(laserOdometryTrans);
  }

  void adjustOutlierCloud()
  {
    PointType point;
    int cloudSize = outlierCloud->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
      point.x         = outlierCloud->points[i].y;
      point.y         = outlierCloud->points[i].z;
      point.z         = outlierCloud->points[i].x;
      point.intensity = outlierCloud->points[i].intensity;

      outlierCloud->points[i] = point;
    }
  }

  void publishCloudsLast()
  {
    updateImuRollPitchYawStartSinCos();

    int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++)
    {
      TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
    }

    int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
    for (int i = 0; i < surfPointsLessFlatNum; i++)
    {
      TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
    }

    pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp                          = laserCloudCornerLast;
    laserCloudCornerLast                           = laserCloudTemp;

    laserCloudTemp     = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum   = laserCloudSurfLast->points.size();

    if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
    {
      kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
      kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
    }

    frameCount++;

    if (frameCount >= skipFrameNum + 1)
    {
      frameCount = 0;

      adjustOutlierCloud();
      sensor_msgs::PointCloud2 outlierCloudLast2;
      pcl::toROSMsg(*outlierCloud, outlierCloudLast2);
      outlierCloudLast2.header.stamp    = cloudHeader.stamp;
      outlierCloudLast2.header.frame_id = "/camera";
      pubOutlierCloudLast.publish(outlierCloudLast2);

      sensor_msgs::PointCloud2 laserCloudCornerLast2;
      pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
      laserCloudCornerLast2.header.stamp    = cloudHeader.stamp;
      laserCloudCornerLast2.header.frame_id = "/camera";
      pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

      sensor_msgs::PointCloud2 laserCloudSurfLast2;
      pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
      laserCloudSurfLast2.header.stamp    = cloudHeader.stamp;
      laserCloudSurfLast2.header.frame_id = "/camera";
      pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
    }
  }

  void runFeatureAssociation()
  {
    if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud
        && std::abs(timeNewSegmentedCloudInfo - timeNewSegmentedCloud) < 0.05
        && std::abs(timeNewOutlierCloud - timeNewSegmentedCloud) < 0.05)
    {
      newSegmentedCloud     = false;
      newSegmentedCloudInfo = false;
      newOutlierCloud       = false;
    }
    else
    {
      return;
    }
    /**
        1. Feature Extraction
    */
    adjustDistortion(); // 坐标系转换

    calculateSmoothness(); // 计算曲率

    markOccludedPoints(); // 标记在水平扫描方向上，屏蔽不可靠点

    extractFeatures(); // 特征提取包括，角点和面点

    publishCloud(); // cloud for visualization

    /**
    2. Feature Association
    */
    if (!systemInitedLM)
    {
      checkSystemInitialization(); // 仅执行一次，用于初始化
      return;
    }

    updateInitialGuess();

    updateTransformation(); // 计算两帧激光数据之间转换矩阵

    integrateTransformation(); // 迭代更新，将每两帧之间变换，进行累计变换，构建里程计

    publishOdometry();

    publishCloudsLast(); // cloud to mapOptimization
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lego_loam");

  ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");

  FeatureAssociation FA;

  ros::Rate rate(200);
  while (ros::ok())
  {
    ros::spinOnce();

    FA.runFeatureAssociation();

    rate.sleep();
  }

  ros::spin();
  return 0;
}
