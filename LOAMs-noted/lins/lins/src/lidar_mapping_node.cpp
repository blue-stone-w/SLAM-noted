/*

*/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <math_utils.h>
#include <parameters.h>

#include <eigen3/Eigen/Dense>

using namespace gtsam;
using namespace parameter;
using namespace std;

const int imuQueLength_ = 200; // construct a loop queue to limit size of imu data

// A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
struct PointXYZIRPYT
{
  // https://blog.csdn.net/qq_42700518/article/details/104820161
  // https://zhuanlan.zhihu.com/p/372812893
  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_POINT4D PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 内存对齐; make sure our new allocators are aligned
} EIGEN_ALIGN16; // define a struct variable

// User defined point structures can be registered using PCL macros. http://wiki.ros.org/pcl_ros/cturtle
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class MappingHandler
{
 private:
  NonlinearFactorGraph gtSAMgraph;
  Values initialEstimate;
  Values optimizedEstimate;
  ISAM2 *isam;
  Values isamCurrentEstimate;

  noiseModel::Diagonal::shared_ptr priorNoise;
  noiseModel::Diagonal::shared_ptr odometryNoise;
  noiseModel::Diagonal::shared_ptr constraintNoise;

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubKeyPoses;
  ros::Publisher pubOdomXYZAftMapped;

  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pubRecentKeyFrames;

  ros::Subscriber subLaserCloudCornerLast;
  ros::Subscriber subLaserCloudSurfLast;
  ros::Subscriber subOutlierCloudLast;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subImu;

  tf::TransformBroadcaster tfBroadcaster;
  nav_msgs::Odometry odomAftMapped; // 最终的里程计位姿
  tf::StampedTransform aftMappedTrans; // 最终的tf变换

  tf::TransformBroadcaster tfXYZBroadcaster;
  nav_msgs::Odometry odomXYZAftMapped;
  tf::StampedTransform aftMappedXYZTrans;

  vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
  vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
  vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;

  deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
  int latestFrameID; // use in part of save recent frames, to judge whether this frame is saved in "recent frames"

  vector<int> surroundingExistingKeyPosesID; // 某半径内关键帧位姿下采样后的位姿ID;这个点云ID可以用于更新地图
  deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
  deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

  PointType previousRobotPosPoint; // 上一关键帧的位姿
  PointType currentRobotPosPoint;

  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  // 历史关键帧的位置
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

  pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
  pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;
  pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOutlierLastDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLast; // surf and outlier
  pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS;

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud; // near to loop frame; map坐标系下，回环帧附近点云
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

  // 回环中使用的是ICP，不需要区分corner和surf。因此实际上只使用了surfkeycloud，而且没有对surfkey执行降采样
  pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis; // square distance

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;
  pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames;
  pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;
  pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
  pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;

  double timeLaserCloudCornerLast;
  double timeLaserCloudSurfLast;
  double timeLaserOdometry; // 来自lins——fusion
  double timeLaserCloudOutlierLast;
  double timeLastGloalMapPublish;

  bool newLaserCloudCornerLast;
  bool newLaserCloudSurfLast;
  bool newLaserOdometry;
  bool newLaserCloudOutlierLast;

  // get from scam-map optimizated transform TobeMapped; 全局优化后也会被更新
  float transformLast[6];
  // (,,,,,)transform from laser odometry(lins_fusion) (whether correction???); odo回调保存的位置，即当前帧的里程计位姿
  float transformSum[6];
  // 连续两帧里程计位姿之间的相对位姿
  float transformIncre[6];
  // initial value get from transform AftMapped; use and optimize in scam-map optimization; 当前帧不断被优化的位姿，全局优化后也会被更新
  float transformTobeMapped[6];
  // get from old transformsum; 上一帧的里程计位姿
  float transformBefMapped[6];
  // (,,,x,y,z), initial value get from scam-map optimizated transform TobeMapped; optimize with isam;上一帧的最优位姿，全局优化后也会被更新
  float transformAftMapped[6];

  int imuPointerFront;
  int imuPointerLast;

  double imuTime[imuQueLength_]; // 缓存IMU消息的循环队列
  float imuRoll[imuQueLength_];
  float imuPitch[imuQueLength_];

  std::mutex mtx;

  double timeLastProcessing;

  PointType pointOri, pointSel, pointProj, coeff;

  cv::Mat matA0; // AX=B
  cv::Mat matB0;
  cv::Mat matX0;

  cv::Mat matA1; // square size and be symmetrical
  cv::Mat matD1; // save eigenvalues
  cv::Mat matV1; // save eigenvectors

  bool isDegenerate;
  cv::Mat matP; // 在迭代过程中，由于退化，位姿的某些分量不再更新，这个矩阵用于不更新这部分分量。

  int laserCloudCornerFromMapDSNum;
  int laserCloudSurfFromMapDSNum;
  int laserCloudCornerLastDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudOutlierLastDSNum;
  int laserCloudSurfTotalLastDSNum;

  bool potentialLoopFlag; // 是否构成回环
  double timeSaveFirstCurrentScanForLoopClosure; // 构成回环时，大致视作当前帧的时间
  int closestHistoryFrameID;
  int latestFrameIDLoopCloure;

  bool aLoopIsClosed; // true表示找到回环,需要执行一次图优化

  // c: corner, s: surf, t: translation
  float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ; // get from transformTobeMapped
  float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ; // get from cloudKeyPoses6D

 public:
  MappingHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh) :
    nh(nh), pnh(pnh)
  {
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip      = 1;
    isam                            = new ISAM2(parameters);

    pubKeyPoses           = pnh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
    pubLaserCloudSurround = pnh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
    pubOdomAftMapped      = pnh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);
    pubOdomXYZAftMapped   = pnh.advertise<nav_msgs::Odometry>("/aft_xyz_mapped_to_init", 5);

    // 接收点云并标记标志位
    subLaserCloudCornerLast = pnh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &MappingHandler::laserCloudCornerLastHandler, this);
    subLaserCloudSurfLast   = pnh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &MappingHandler::laserCloudSurfLastHandler, this);
    subOutlierCloudLast     = pnh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2, &MappingHandler::laserCloudOutlierLastHandler, this);
    // 接收位姿，姿态转换成rpy保存到transformSum，标记标志位; lins_fusion发布
    subLaserOdometry = pnh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &MappingHandler::laserOdometryHandler, this);
    subImu           = pnh.subscribe<sensor_msgs::Imu>("no_imu", 50, &MappingHandler::imuHandler, this);

    pubHistoryKeyFrames = pnh.advertise<sensor_msgs::PointCloud2>("/history_cloud", 2);
    pubIcpKeyFrames     = pnh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);
    pubRecentKeyFrames  = pnh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);

    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

    downSizeFilterHistoryKeyFrames.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0);

    downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0);
    downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4);

    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.child_frame_id  = "/aft_mapped";

    // tf变换
    aftMappedTrans.frame_id_       = "/camera_init";
    aftMappedTrans.child_frame_id_ = "/aft_mapped";

    aftMappedXYZTrans.frame_id_       = "/map";
    aftMappedXYZTrans.child_frame_id_ = "/aft_xyz_mapped";

    allocateMemory( );
  }

  void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    timeLaserCloudCornerLast = msg->header.stamp.toSec( );
    laserCloudCornerLast->clear( );
    pcl::fromROSMsg(*msg, *laserCloudCornerLast);
    newLaserCloudCornerLast = true;
  }

  void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    timeLaserCloudSurfLast = msg->header.stamp.toSec( );
    laserCloudSurfLast->clear( );
    pcl::fromROSMsg(*msg, *laserCloudSurfLast);
    newLaserCloudSurfLast = true;
  }

  void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    timeLaserCloudOutlierLast = msg->header.stamp.toSec( );
    laserCloudOutlierLast->clear( );
    pcl::fromROSMsg(*msg, *laserCloudOutlierLast);
    newLaserCloudOutlierLast = true;
  }

  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
  {
    // laserOdometry is estimator -> globalStateYZX_
    timeLaserOdometry = laserOdometry->header.stamp.toSec( );
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
    transformSum[0]  = -pitch;
    transformSum[1]  = -yaw;
    transformSum[2]  = roll;
    transformSum[3]  = laserOdometry->pose.pose.position.x;
    transformSum[4]  = laserOdometry->pose.pose.position.y;
    transformSum[5]  = laserOdometry->pose.pose.position.z;
    newLaserOdometry = true;
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn)
  {
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation); // extract data from imu in to orientation
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    imuPointerLast           = (imuPointerLast + 1) % imuQueLength_; //
    imuTime[imuPointerLast]  = imuIn->header.stamp.toSec( );
    imuRoll[imuPointerLast]  = roll;
    imuPitch[imuPointerLast] = pitch;
  }

  void allocateMemory( )
  { // allocate memory and reset varibles
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>( ));
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>( ));

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>( ));
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>( ));

    surroundingKeyPoses.reset(new pcl::PointCloud<PointType>( ));
    surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>( ));

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>( ));
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>( ));
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>( ));
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>( ));
    laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>( ));
    laserCloudOutlierLastDS.reset(new pcl::PointCloud<PointType>( ));
    laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>( ));
    laserCloudSurfTotalLastDS.reset(new pcl::PointCloud<PointType>( ));

    laserCloudOri.reset(new pcl::PointCloud<PointType>( ));
    coeffSel.reset(new pcl::PointCloud<PointType>( ));

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>( ));
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>( ));
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>( ));
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>( ));

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>( ));
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>( ));

    nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>( ));
    nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>( ));
    nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>( ));
    nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>( ));

    latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>( ));
    latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>( ));
    latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>( ));

    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>( ));
    globalMapKeyPoses.reset(new pcl::PointCloud<PointType>( ));
    globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>( ));
    globalMapKeyFrames.reset(new pcl::PointCloud<PointType>( ));
    globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>( ));

    timeLaserCloudCornerLast  = 0;
    timeLaserCloudSurfLast    = 0;
    timeLaserOdometry         = 0;
    timeLaserCloudOutlierLast = 0;
    timeLastGloalMapPublish   = 0;

    timeLastProcessing = -1;

    newLaserCloudCornerLast = false; // "false" means all corner have been processed
    newLaserCloudSurfLast   = false;

    newLaserOdometry         = false;
    newLaserCloudOutlierLast = false;

    for (int i = 0; i < 6; ++i)
    {
      transformLast[i]       = 0;
      transformSum[i]        = 0;
      transformIncre[i]      = 0;
      transformTobeMapped[i] = 0;
      transformBefMapped[i]  = 0;
      transformAftMapped[i]  = 0;
    }

    imuPointerFront = 0;
    imuPointerLast  = -1;

    for (int i = 0; i < imuQueLength_; ++i)
    {
      imuTime[i]  = 0;
      imuRoll[i]  = 0;
      imuPitch[i] = 0;
    }

    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise    = noiseModel::Diagonal::Variances(Vector6); // high confidence;
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);

    matA0 = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
    matB0 = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));
    matX0 = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));

    matA1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matD1 = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    matV1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

    isDegenerate = false;
    matP         = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    laserCloudCornerFromMapDSNum = 0;
    laserCloudSurfFromMapDSNum   = 0;
    laserCloudCornerLastDSNum    = 0;
    laserCloudSurfLastDSNum      = 0;
    laserCloudOutlierLastDSNum   = 0;
    laserCloudSurfTotalLastDSNum = 0;

    potentialLoopFlag = false;
    aLoopIsClosed     = false;

    latestFrameID = 0;
  }

  void loopClosureThread( )
  {
    if (loopClosureEnableFlag == false) return;

    ros::Rate rate(1); // frequence; 每1s进行一次回环检测，比地图刷新频率要低
    while (ros::ok( ))
    {
      rate.sleep( );
      performLoopClosure( );
    }
  }

  // 每间隔1s对最新的关键帧，做回环检测和匹配
  void performLoopClosure( )
  {
    if (cloudKeyPoses3D->points.empty( ) == true) return;

    if (potentialLoopFlag == false)
    {
      // find valid loop; 查找最后一个关键帧的最近邻，以第一个30s以前的关键帧为中心累加地图
      if (detectLoopClosure( ) == true)
      {
        potentialLoopFlag                      = true;
        timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
      }
      if (potentialLoopFlag == false) return;
    }

    potentialLoopFlag = false; // 复位

    // 以下，将最后关键帧与回环帧匹配
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100); // 设置对应点容忍最大距离
    icp.setMaximumIterations(100); // 最大迭代次数
    icp.setTransformationEpsilon(1e-6); // 两次变化矩阵之间的差值
    icp.setEuclideanFitnessEpsilon(1e-6); // 设置收敛条件是均方误差和小于阈值， 停止迭代
    icp.setRANSACIterations(0); // 不进行RANSAC迭代

    icp.setInputSource(latestSurfKeyFrameCloud);
    icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>( )); // 变换后的源点云
    icp.align(*unused_result); // Perform the alignment; 点云匹配，计算最近关键帧的漂移

    // 不成功或者得分比较低，直接返回
    if (icp.hasConverged( ) == false || icp.getFitnessScore( ) > historyKeyframeFitnessScore) return;

    // 偏移累加到最新关键帧点云上，发布
    if (pubIcpKeyFrames.getNumSubscribers( ) != 0)
    {
      pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>( ));
      // too many arguments ###
      pcl::transformPointCloud(*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation( ));
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
      cloudMsgTemp.header.stamp    = ros::Time( ).fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubIcpKeyFrames.publish(cloudMsgTemp);
    }

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionCameraFrame;
    correctionCameraFrame = icp.getFinalTransformation( );
    pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
    Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch); // 漂移量
    Eigen::Affine3f tWrong               = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]); // 旧的位姿
    Eigen::Affine3f tCorrect             = correctionLidarFrame * tWrong; // 新的位姿
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z)); // corrected cur pose
    gtsam::Pose3 poseTo   = pclPointTogtsamPose3(cloudKeyPoses6D->points[closestHistoryFrameID]); // corrected prev pose; 回环帧位姿
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore( );
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    constraintNoise = noiseModel::Diagonal::Variances(Vector6); // 6维: gtsam中角度是以欧拉角的形式优化的

    // 以上点云匹配的过程中不会锁。以下开始锁住，scan2map优化会堵塞，因为都会对因子图进行修改
    std::lock_guard<std::mutex> lock(mtx); // 离开局部作用域，析构函数自动完成解锁功能
    // 在当前的图中新增一个回环约束
    gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise));
    isam->update(gtSAMgraph);
    isam->update( );
    // clear constraint and vertice(they have been add to isam, so clearing them won't influence optimization)
    gtSAMgraph.resize(0);

    aLoopIsClosed = true; // 标志位，在scan2map函数中真正将图优化结果更新过去
    // 执行完成，释放锁，scan2map优化环节才能正常进行，并将这里优化的结果同步过去
    // 也就是说对关键帧位姿的修改是都是在scan2map优化线程中进行的
  }

  bool detectLoopClosure( )
  {
    latestSurfKeyFrameCloud->clear( );
    nearHistorySurfKeyFrameCloud->clear( );
    nearHistorySurfKeyFrameCloudDS->clear( );

    // scan2map优化中也存在这个锁，两者不同时进行（scan2map会更新currentRobotPosPoint）
    std::lock_guard<std::mutex> lock(mtx);

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    // 从最后的关键帧搜索最近邻
    kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    closestHistoryFrameID = -1;
    for (int i = 0; i < pointSearchIndLoop.size( ); ++i)
    {
      int id = pointSearchIndLoop[i];
      // dont loop with recent frame;
      if (abs(cloudKeyPoses6D->points[id].time - timeLaserOdometry) > 30.0)
      { // 第一个差距30s以上的近邻(时间近的近邻)
        closestHistoryFrameID = id;
        break;
      }
    }
    if (closestHistoryFrameID == -1) return false;

    // 最近关键帧的点云投影到map坐标系下
    latestFrameIDLoopCloure = cloudKeyPoses3D->points.size( ) - 1;
    // 回环匹配使用ICP，没有必要分为两个点云来匹配，因此全部放入 latest Surf Key Frame Cloud
    // transfrom frame from lidar coordinate to global/map coordinate
    *latestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
    *latestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);

    pcl::PointCloud<PointType>::Ptr hahaCloud(new pcl::PointCloud<PointType>( )); // temperary varible to remove non points
    int cloudSize = latestSurfKeyFrameCloud->points.size( );
    for (int i = 0; i < cloudSize; ++i)
    {
      // intensity < 0 means nan points
      if ((int)latestSurfKeyFrameCloud->points[i].intensity >= 0)
      {
        hahaCloud->push_back(latestSurfKeyFrameCloud->points[i]);
      }
    }
    latestSurfKeyFrameCloud->clear( );
    *latestSurfKeyFrameCloud = *hahaCloud; // have remove non points

    // 回环帧投影到map坐标系下；回环帧由当前帧及之前的若干帧点云组成
    for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j)
    {
      if (closestHistoryFrameID + j < 0 || closestHistoryFrameID + j > latestFrameIDLoopCloure) continue;
      *nearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[closestHistoryFrameID + j],
                                                            &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
      *nearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[closestHistoryFrameID + j],
                                                            &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
    }

    downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
    downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);

    if (pubHistoryKeyFrames.getNumSubscribers( ) != 0)
    {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
      cloudMsgTemp.header.stamp    = ros::Time( ).fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubHistoryKeyFrames.publish(cloudMsgTemp);
    }
    return true;
  }

  pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn)
  {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>( ));

    PointType *pointFrom;
    PointType pointTo;
    int cloudSize = cloudIn->points.size( );
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i)
    {
      pointFrom = &cloudIn->points[i];
      // rotate around z-axis
      float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
      float y1 = stYaw * pointFrom->x + ctYaw * pointFrom->y;
      float z1 = pointFrom->z;
      // rotate around x-axis
      float x2 = x1;
      float y2 = ctRoll * y1 - stRoll * z1;
      float z2 = stRoll * y1 + ctRoll * z1;
      // rotate around z-axis and translation
      pointTo.x         = ctPitch * x2 + stPitch * z2 + tInX;
      pointTo.y         = y2 + tInY;
      pointTo.z         = -stPitch * x2 + ctPitch * z2 + tInZ;
      pointTo.intensity = pointFrom->intensity;

      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

  pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
  {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>( ));

    PointType *pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size( );
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i)
    {
      pointFrom = &cloudIn->points[i];
      // x2 = Rx1 + t;
      float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
      float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw) * pointFrom->y;
      float z1 = pointFrom->z;

      float x2 = x1;
      float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
      float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

      pointTo.x         = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
      pointTo.y         = y2 + transformIn->y;
      pointTo.z         = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
      pointTo.intensity = pointFrom->intensity;

      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

  Eigen::Affine3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint)
  {
    return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
  }
  Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
  {
    return Pose3(Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll), double(thisPoint.pitch)),
                 Point3(double(thisPoint.z), double(thisPoint.x), double(thisPoint.y)));
  }

  void visualizeGlobalMapThread( )
  {
    ros::Rate rate(0.2);
    while (ros::ok( ))
    {
      rate.sleep( );
      publishGlobalMap( );
    }
  }

  // 根据关键帧位姿和点云，发布全局地图
  void publishGlobalMap( )
  {
    if (pubLaserCloudSurround.getNumSubscribers( ) == 0) return;
    if (cloudKeyPoses3D->points.empty( ) == true) return;

    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;

    mtx.lock( );
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock( );

    for (int i = 0; i < pointSearchIndGlobalMap.size( ); ++i)
    {
      globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    }

    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

    for (int i = 0; i < globalMapKeyPosesDS->points.size( ); ++i)
    {
      // downsized intensity may not integer, so use (int) to force convert data type
      int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
      *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
      *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
      *globalMapKeyFrames += *transformPointCloud(outlierCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    }

    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp    = ros::Time( ).fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubLaserCloudSurround.publish(cloudMsgTemp);

    globalMapKeyPoses->clear( );
    globalMapKeyPosesDS->clear( );
    globalMapKeyFrames->clear( );
    globalMapKeyFramesDS->clear( );
  }

  int lidarCounter = 0; // run函数运行中处理过的点云的数量
  double duration_ = 0; // run函数处理点云的平均用时
  // 200Hz的频率刷新，远大于雷达频率，能够保证数据能及时被处理
  // 这里要保证函数整体运行的要足够快，避免没有处理完就被回调覆盖
  void run( )
  {
    if (newLaserCloudCornerLast && std::abs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 && newLaserCloudSurfLast && std::abs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 && newLaserCloudOutlierLast && std::abs(timeLaserCloudOutlierLast - timeLaserOdometry) < 0.005 && newLaserOdometry)
    { // 凑够一组，就处理一次
      newLaserCloudCornerLast  = false;
      newLaserCloudSurfLast    = false;
      newLaserCloudOutlierLast = false;
      newLaserOdometry         = false;

      std::lock_guard<std::mutex> lock(mtx); // 这里锁还在回环检测出现

      // 不超过300ms，跳过
      if (timeLaserOdometry - timeLastProcessing >= mappingProcessInterval)
      {
        TicToc ts_total; // 计时类 defined in parameters.h
        timeLastProcessing = timeLaserOdometry;

        // 根据上一帧的偏移量，计算当前最优位姿估计，其实也就是把scan2scan的结果在这里积分
        transformAssociateToMap( );
        // 搜索周围关键帧，并拼接成map，对待是否回环存在不同的处理方式
        extractSurroundingKeyFrames( );

        downsampleCurrentScan( );
        // use L-m optimization scan2map位姿优化，更新transformBefMapped，transformAftMapped, currentRobotPosPoint
        scan2MapOptimization( );
        // 判断是否需要新增关键帧，如果需要那么利用因子图进行全局位姿优化，并新增关键帧
        saveKeyFramesAndFactor( );
        // 判断在另一个线程中是否形成了回环, 只有形成回环，那么才会在这里更新所有关键帧的pose
        // 注意这里一个细节是，如果回环先完成的，那么新增加的节点不会被回环优化，因此不会被改动
        correctPoses( ); // if loop, should adjust pose between this loop

        publishTF( ); // 发布最终的odo和tf变换
        publishXYZTF( ); // 新的坐标习惯再发布一次
        publishKeyPosesAndFrames( ); // 发布所有的关键帧位置，以及周围的面点地图
        clearCloud( ); // 清除局部的周围地图

        double time_total = ts_total.toc( ); // duration of this "run"
        if (VERBOSE)
        {
          duration_ = (duration_ * lidarCounter + time_total) / (lidarCounter + 1); // average
          lidarCounter++;
          // std::cout << "Mapping: time: " << duration_ << std::endl;
        }
      }
    }
  }

  // 根据上一帧，里程计的偏移量，优化当前里程计位姿???
  void transformAssociateToMap( )
  {
    // rotate around y-axiz and translation
    float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    // rotate around x-axiz
    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;
    // rotate around z-axiz
    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;

    // sbcx: surf,before optimization current
    // calx: corner,after optimization last
    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    /******************************/
    float srx              = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
    transformTobeMapped[0] = -asin(srx);

    float srycrx           = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
    float crycrx           = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
    transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                   crycrx / cos(transformTobeMapped[0]));

    float srzcrx           = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    float crzcrx           = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                   crzcrx / cos(transformTobeMapped[0]));
    /*****************************************************/
    // rotate around z-axis
    x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
    y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];
    // rotate around x-axis
    x2 = x1;
    y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;
    // rotate around y-axis and translation
    transformTobeMapped[3] = transformAftMapped[3] - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
    transformTobeMapped[4] = transformAftMapped[4] - y2;
    transformTobeMapped[5] = transformAftMapped[5] - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
  }

  void extractSurroundingKeyFrames( )
  {
    if (cloudKeyPoses3D->points.empty( ) == true) return; // 如果没有历史关键帧
    // if enable loop, should save key frames to detect loop; if not
    if (loopClosureEnableFlag == true)
    { // 如果开了回环检测
      // 如果近邻不够多，按照时间逆序重新累积周围地图
      if (recentCornerCloudKeyFrames.size( ) < surroundingKeyframeSearchNum)
      {
        recentCornerCloudKeyFrames.clear( );
        recentSurfCloudKeyFrames.clear( );
        recentOutlierCloudKeyFrames.clear( );
        int numPoses = cloudKeyPoses3D->points.size( );
        for (int i = numPoses - 1; i >= 0; --i)
        { // traverse from new to old
          int thisKeyInd                   = (int)cloudKeyPoses3D->points[i].intensity;
          PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
          updateTransformPointCloudSinCos(&thisTransformation);
          // old frame is on the front, new frame stay back; 根据坐标将雷达点云转换到地图坐标系下
          recentCornerCloudKeyFrames.push_front(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
          recentSurfCloudKeyFrames.push_front(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
          recentOutlierCloudKeyFrames.push_front(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
          if (recentCornerCloudKeyFrames.size( ) >= surroundingKeyframeSearchNum) break;
        }
      }
      else
      {
        // 如果cloudKeyPoses3D中有更新的位姿，周围地图更新一帧
        if (latestFrameID != cloudKeyPoses3D->points.size( ) - 1)
        {
          recentCornerCloudKeyFrames.pop_front( );
          recentSurfCloudKeyFrames.pop_front( );
          recentOutlierCloudKeyFrames.pop_front( );
          latestFrameID                    = cloudKeyPoses3D->points.size( ) - 1;
          PointTypePose thisTransformation = cloudKeyPoses6D->points[latestFrameID];
          updateTransformPointCloudSinCos(&thisTransformation);
          recentCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
          recentSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[latestFrameID]));
          recentOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
        }
      }

      // 累加成周围的点云地图
      // clear "recent key frame" in function "correctposes", so there isnt same frames in lccfm
      for (int i = 0; i < recentCornerCloudKeyFrames.size( ); ++i)
      {
        *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
        *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
        *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i]; // 把被排除的点也放入surf构成地图，使地图的点云更完整
      }
    }
    else
    {
      surroundingKeyPoses->clear( );
      surroundingKeyPosesDS->clear( );

      kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
      kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);

      for (int i = 0; i < pointSearchInd.size( ); ++i)
      {
        surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
      }
      downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
      downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS); // 点的强度的整数部分为其在cloudKeyPoses3D中的索引

      // 以下两个大循环，如果surroundingExistingKeyPosesID还存在但是不需要了就删除；
      // 如果需要，但是surroundingExistingKeyPosesID中没有的需要新增
      // 目的是通过增减，更新周围地图
      int numSurroundingPosesDS = surroundingKeyPosesDS->points.size( );
      for (int i = 0; i < surroundingExistingKeyPosesID.size( ); ++i)
      {
        bool existingFlag = false; // "true": this frame exit in "surrounding Existing KeyPoses ID" and "surroundingKeyPosesDS"
        // 判断第surroundingExistingKeyPosesID[i]帧是否还需要
        for (int j = 0; j < numSurroundingPosesDS; ++j)
        {
          if (surroundingExistingKeyPosesID[i] == (int)surroundingKeyPosesDS->points[j].intensity)
          {
            existingFlag = true;
            break;
          }
        }
        if (existingFlag == false)
        { // remove frame in "surrounding Existing KeyPoses ID" and not in "surroundingKeyPosesDS"
          surroundingExistingKeyPosesID.erase(surroundingExistingKeyPosesID.begin( ) + i);
          surroundingCornerCloudKeyFrames.erase(surroundingCornerCloudKeyFrames.begin( ) + i);
          surroundingSurfCloudKeyFrames.erase(surroundingSurfCloudKeyFrames.begin( ) + i);
          surroundingOutlierCloudKeyFrames.erase(surroundingOutlierCloudKeyFrames.begin( ) + i);
          --i; // 删除本位后，与++i抵消
        }
      }
      // now all frames in "surrounding Existing KeyPoses ID" exist in "surroundingKeyPosesDS"

      for (int i = 0; i < numSurroundingPosesDS; ++i)
      {
        bool existingFlag = false;
        for (auto iter = surroundingExistingKeyPosesID.begin( ); iter != surroundingExistingKeyPosesID.end( ); ++iter)
        {
          if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity)
          {
            existingFlag = true;
            break;
          }
        }
        if (existingFlag == true)
        {
          continue;
        }
        else
        { // if this frame in "Surrounding PosesDS" and not in "surrounding ExistingKey PosesID"
          int thisKeyInd                   = (int)surroundingKeyPosesDS->points[i].intensity;
          PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
          updateTransformPointCloudSinCos(&thisTransformation);
          surroundingExistingKeyPosesID.push_back(thisKeyInd);
          surroundingCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
          surroundingSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
          surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
        }
      }

      for (int i = 0; i < surroundingExistingKeyPosesID.size( ); ++i)
      {
        *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
        *laserCloudSurfFromMap += *surroundingSurfCloudKeyFrames[i];
        *laserCloudSurfFromMap += *surroundingOutlierCloudKeyFrames[i];
      }
    }

    // 对周围地图降采样
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size( );

    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size( );
  }

  void updateTransformPointCloudSinCos(PointTypePose *tIn)
  { // 6D transform
    ctRoll = cos(tIn->roll);
    stRoll = sin(tIn->roll);

    ctPitch = cos(tIn->pitch);
    stPitch = sin(tIn->pitch);

    ctYaw = cos(tIn->yaw);
    stYaw = sin(tIn->yaw);

    tInX = tIn->x;
    tInY = tIn->y;
    tInZ = tIn->z;
  }

  void downsampleCurrentScan( )
  {
    laserCloudCornerLastDS->clear( );
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size( );

    laserCloudSurfLastDS->clear( );
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size( );

    laserCloudOutlierLastDS->clear( );
    downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
    downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
    laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size( );

    laserCloudSurfTotalLast->clear( );
    laserCloudSurfTotalLastDS->clear( );
    *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
    *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
    downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
    downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
    laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size( );
  }

  void scan2MapOptimization( )
  {
    if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100)
    {
      kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
      kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

      for (int iterCount = 0; iterCount < 10; iterCount++)
      {
        laserCloudOri->clear( );
        coeffSel->clear( );
        // 求解残差和雅可比
        cornerOptimization(iterCount);
        surfOptimization(iterCount);
        if (LMOptimization(iterCount) == true) break; // 判断是否converge，并对delta做修正
      }
      transformUpdate( ); // 更新transformBefMapped，transformAftMapped
    }
  }

  // 用角点对位姿进行优化
  void cornerOptimization(int iterCount)
  {
    updatePointAssociateToMapSinCos( ); // get 6D pose from transformTobeMapped; 根据上一轮优化的位姿，更新一些中间变量
    for (int i = 0; i < laserCloudCornerLastDSNum; i++)
    {
      pointOri = laserCloudCornerLastDS->points[i];
      pointAssociateToMap(&pointOri, &pointSel); // transform point from lidar coordinate to global/map coordinate
      kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

      if (pointSearchSqDis[4] < 1.0)
      { // this point is most far point; 如果最远的最近邻足够接近
        float cx = 0, cy = 0, cz = 0; // center of 5 closest points
        for (int j = 0; j < 5; j++)
        {
          // laserCloudCornerFromMapDS与kdtreeCornerFromMap是对应的
          cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
          cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
          cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
        }
        cx /= 5;
        cy /= 5;
        cz /= 5; // 求中心

        float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
        for (int j = 0; j < 5; j++)
        {
          float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
          float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
          float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

          a11 += ax * ax;
          a12 += ax * ay;
          a13 += ax * az;
          a22 += ay * ay;
          a23 += ay * az;
          a33 += az * az;
        }
        a11 /= 5;
        a12 /= 5;
        a13 /= 5;
        a22 /= 5;
        a23 /= 5;
        a33 /= 5;

        // 协方差矩阵
        matA1.at<float>(0, 0) = a11;
        matA1.at<float>(0, 1) = a12;
        matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12;
        matA1.at<float>(1, 1) = a22;
        matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13;
        matA1.at<float>(2, 1) = a23;
        matA1.at<float>(2, 2) = a33;

        cv::eigen(matA1, matD1, matV1); // D1 is eigenvalues in the descending order; V1 is eigenvector

        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
        {
          float x0 = pointSel.x; // define x0,y0,z0 as point O
          float y0 = pointSel.y;
          float z0 = pointSel.z;
          float x1 = cx + 0.1 * matV1.at<float>(0, 0); // two points in corresponding edge
          float y1 = cy + 0.1 * matV1.at<float>(0, 1); // define x1,y1,z1 as point A
          float z1 = cz + 0.1 * matV1.at<float>(0, 2);
          float x2 = cx - 0.1 * matV1.at<float>(0, 0); // define x2,y2,z2 as point B
          float y2 = cy - 0.1 * matV1.at<float>(0, 1);
          float z2 = cz - 0.1 * matV1.at<float>(0, 2);

          // define MO = cross multiplication of AO and BO; |MO|=a012
          float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
                            + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
                            + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));
          // distance between two points = |AB|; 线段长度
          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
          float la  = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;
          float lb  = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
          float lc  = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
          float ld2 = a012 / l12; // residual: distance between point O and edge AB

          // Kernel function to reduce weight of residual(large residual-->small weight)
          float s = 1 - 0.9 * fabs(ld2); // s>0, because distance(this point,nearest point) < 1

          coeff.x         = s * la;
          coeff.y         = s * lb;
          coeff.z         = s * lc;
          coeff.intensity = s * ld2;

          // this is a valid constraint for map optimization; residual < 10cm
          if (s > 0.1)
          {
            laserCloudOri->push_back(pointOri); // xyz is jacobi direction, intensity is residual
            coeffSel->push_back(coeff); // this constraint is valid
          }
        }
      }
    }
  }

  void updatePointAssociateToMapSinCos( )
  {
    cRoll = cos(transformTobeMapped[0]);
    sRoll = sin(transformTobeMapped[0]);

    cPitch = cos(transformTobeMapped[1]);
    sPitch = sin(transformTobeMapped[1]);

    cYaw = cos(transformTobeMapped[2]);
    sYaw = sin(transformTobeMapped[2]);

    tX = transformTobeMapped[3];
    tY = transformTobeMapped[4];
    tZ = transformTobeMapped[5];
  }

  void pointAssociateToMap(PointType const *const pi, PointType *const po)
  {
    float x1 = cYaw * pi->x - sYaw * pi->y;
    float y1 = sYaw * pi->x + cYaw * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cRoll * y1 - sRoll * z1;
    float z2 = sRoll * y1 + cRoll * z1;

    po->x         = cPitch * x2 + sPitch * z2 + tX;
    po->y         = y2 + tY;
    po->z         = -sPitch * x2 + cPitch * z2 + tZ;
    po->intensity = pi->intensity;
  }

  void surfOptimization(int iterCount)
  {
    updatePointAssociateToMapSinCos( ); // get 6D pose from transformTobeMapped
    for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++)
    {
      pointOri = laserCloudSurfTotalLastDS->points[i];
      pointAssociateToMap(&pointOri, &pointSel); // transform point from lidar coordinate to global coordinate
      kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

      if (pointSearchSqDis[4] < 1.0)
      {
        for (int j = 0; j < 5; j++)
        {
          matA0.at<float>(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
          matA0.at<float>(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
          matA0.at<float>(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
        }
        // solve superdefinite equation: AX = B; x is nomal vector of this plane
        // The solution (matrix inversion) method is QR factorization;
        // the system can be over-defined and/or the matrix src1 can be singular
        cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

        // ax+by+cz+d=0的四个参数
        float pa = matX0.at<float>(0, 0);
        float pb = matX0.at<float>(1, 0);
        float pc = matX0.at<float>(2, 0);
        float pd = 1;

        // normalize x --> |x| = 1; 归一化
        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++)
        {
          // if one point of 5 points is far from this plane, curvature of this plane is large and this is a invalid palne
          if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x
                   + pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y
                   + pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd)
              > 0.2)
          {
            planeValid = false;
            break;
          }
        }

        if (planeValid)
        {
          // residual: distance between point and plane
          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

          // |pointSel| is large, surf feature is more important than corner
          float s         = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
          coeff.x         = s * pa;
          coeff.y         = s * pb;
          coeff.z         = s * pc;
          coeff.intensity = s * pd2;
          if (s > 0.1)
          {
            laserCloudOri->push_back(pointOri);
            coeffSel->push_back(coeff);
          }
        }
      }
    }
  }

  bool LMOptimization(int iterCount)
  {
    // calculate sin cos of rotation around xyz-axis
    float srx = sin(transformTobeMapped[0]);
    float crx = cos(transformTobeMapped[0]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[2]);
    float crz = cos(transformTobeMapped[2]);

    int laserCloudSelNum = laserCloudOri->points.size( );
    if (laserCloudSelNum < 50) return false; // constraints is few

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    for (int i = 0; i < laserCloudSelNum; i++)
    {
      pointOri = laserCloudOri->points[i];
      coeff    = coeffSel->points[i];

      // in camera
      // calculate jacobi of rotation(rotate around yxz in camera <-> around zyx in lidar)
      // roll
      float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;
      // pitch
      float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;
      // yaw
      float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

      // lidar -> camera
      matA.at<float>(i, 0) = arx; // jacobi of rotation
      matA.at<float>(i, 1) = ary;
      matA.at<float>(i, 2) = arz;
      matA.at<float>(i, 3) = coeff.x; // jacobi of translation
      matA.at<float>(i, 4) = coeff.y;
      matA.at<float>(i, 5) = coeff.z;
      matB.at<float>(i, 0) = -coeff.intensity; // residual
    }

    // JTJ * deltax = -JTe (J:Jacobi of e; e:residual, scalar quantity)
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0)
    { // check whether degenerated before first iteration
      cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV); // mat E: eigenvalue, max->min; mat V: eigenvector
      matV.copyTo(matV2);

      isDegenerate      = false;
      float eignThre[6] = {100, 100, 100, 100, 100, 100};
      for (int i = 5; i >= 0; i--)
      { // traverse from min to max
        if (matE.at<float>(0, i) < eignThre[i])
        { // 如果有的特征值小于阈值，其对应的特征向量置为0
          for (int j = 0; j < 6; j++) matV2.at<float>(i, j) = 0;
          isDegenerate = true; // 只有iterCount==0有机会置为true
        }
        else
        {
          break;
        }
      }
      matP = matV.inv( ) * matV2; // eigenvector matrix without degenerated component
    }
    // for example, in a long corridor, cant estimate translation in direction of corridor,
    // thus, corresponding vector is degenerated and corresponding component of delta x should be zero

    if (isDegenerate)
    { // set degenerated component of delta x as zero
      cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) + pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) + pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) + pow(matX.at<float>(4, 0) * 100, 2) + pow(matX.at<float>(5, 0) * 100, 2));
    if (deltaR < 0.05 && deltaT < 0.05) return true; // 如果delta足够小, pose has converged, end optimization
    return false; // pose hasn't converged, keep optimizing
  }

  // weighted fuse transformTobeMapped and imu
  void transformUpdate( )
  {
    if (imuPointerLast >= 0)
    { // imu data isnt empty
      // roll and pitch is observable, so weighted fuse pose from lidar estimated and magnetometer
      float imuRollLast = 0, imuPitchLast = 0;
      while (imuPointerFront != imuPointerLast)
      { // there is unused imu data
        // first imu data that is later than
        if (timeLaserOdometry + SCAN_PERIOD < imuTime[imuPointerFront]) break;
        imuPointerFront = (imuPointerFront + 1) % imuQueLength_;
      }

      if (timeLaserOdometry + SCAN_PERIOD > imuTime[imuPointerFront])
      { // all imu data is too early
        imuRollLast  = imuRoll[imuPointerFront];
        imuPitchLast = imuPitch[imuPointerFront];
      }
      else
      {
        // first imu data that is earlier than
        int imuPointerBack = (imuPointerFront + imuQueLength_ - 1) % imuQueLength_;
        float ratioFront   = (timeLaserOdometry + SCAN_PERIOD - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        float ratioBack    = (imuTime[imuPointerFront] - timeLaserOdometry - SCAN_PERIOD) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        imuRollLast        = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
        imuPitchLast       = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      }

      transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
      transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
    }

    // 更新transformBefMapped，transformAftMapped
    for (int i = 0; i < 6; i++)
    {
      transformBefMapped[i] = transformSum[i];
      transformAftMapped[i] = transformTobeMapped[i];
    }
  }

  void saveKeyFramesAndFactor( )
  {
    currentRobotPosPoint.x = transformAftMapped[3]; // transformAftMapped已经在scan2map优化中被更新过了
    currentRobotPosPoint.y = transformAftMapped[4];
    currentRobotPosPoint.z = transformAftMapped[5];

    bool saveThisKeyFrame = true;
    // 每0.3m保存一个关键帧
    if (sqrt((previousRobotPosPoint.x - currentRobotPosPoint.x) * (previousRobotPosPoint.x - currentRobotPosPoint.x)
             + (previousRobotPosPoint.y - currentRobotPosPoint.y) * (previousRobotPosPoint.y - currentRobotPosPoint.y)
             + (previousRobotPosPoint.z - currentRobotPosPoint.z) * (previousRobotPosPoint.z - currentRobotPosPoint.z))
        < 0.3)
    {
      saveThisKeyFrame = false;
    }

    if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty( )) return; // 不需要保存且有位姿存在

    previousRobotPosPoint = currentRobotPosPoint; // 更新previousRobotPosPoint为了下次比较

    // 新增关键帧，并进行全局的位姿优化
    if (cloudKeyPoses3D->points.empty( ))
    {
      // 增加先验位姿
      gtSAMgraph.add(PriorFactor<Pose3>(0,
                                        Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                              Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])),
                                        priorNoise));
      initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                      Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
      for (int i = 0; i < 6; ++i) transformLast[i] = transformTobeMapped[i];
    }
    else
    {
      gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                                    Point3(transformLast[5], transformLast[3], transformLast[4]));
      gtsam::Pose3 poseTo   = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                    Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
      // 增加双边约束
      gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size( ) - 1,
                                          cloudKeyPoses3D->points.size( ),
                                          poseFrom.between(poseTo),
                                          odometryNoise)); // cloudKeyPoses3D和value都需要被新增，在后面进行
      // 在value中新增节点
      initialEstimate.insert(cloudKeyPoses3D->points.size( ),
                             Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                   Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
    }

    isam->update(gtSAMgraph, initialEstimate);
    isam->update( );

    // clear constraint and vertice(they have been add to isam, so clearing them won't influence optimization)
    gtSAMgraph.resize(0);
    initialEstimate.clear( );
    // 清除了因子图和被优化的变量，但是优化结果还在

    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate( );
    // optimized pose of newest key frame(index is "size-1")
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size( ) - 1); // 全局优化出来的当前帧的位姿

    thisPose3D.x = latestEstimate.translation( ).y( );
    thisPose3D.y = latestEstimate.translation( ).z( );
    thisPose3D.z = latestEstimate.translation( ).x( );
    // 也就是该点在vector中的索引，这里之所以保存索引是因为在不回环的情况下会对位姿做降采样，需要恢复出位姿对应的索引
    thisPose3D.intensity = cloudKeyPoses3D->points.size( ); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D); // 在cloudKeyPoses3D中新增关键帧

    thisPose6D.x         = thisPose3D.x;
    thisPose6D.y         = thisPose3D.y;
    thisPose6D.z         = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll      = latestEstimate.rotation( ).pitch( ); // different coordinates system in camera and lidar
    thisPose6D.pitch     = latestEstimate.rotation( ).yaw( );
    thisPose6D.yaw       = latestEstimate.rotation( ).roll( );
    thisPose6D.time      = timeLaserOdometry; // because performing optimization in this time
    cloudKeyPoses6D->push_back(thisPose6D);

    // 用全局优化的位姿，再次更新transformAftMapped
    if (cloudKeyPoses3D->points.size( ) > 1)
    {
      transformAftMapped[0] = latestEstimate.rotation( ).pitch( );
      transformAftMapped[1] = latestEstimate.rotation( ).yaw( );
      transformAftMapped[2] = latestEstimate.rotation( ).roll( );
      transformAftMapped[3] = latestEstimate.translation( ).y( );
      transformAftMapped[4] = latestEstimate.translation( ).z( );
      transformAftMapped[5] = latestEstimate.translation( ).x( );

      for (int i = 0; i < 6; ++i)
      {
        transformLast[i]       = transformAftMapped[i];
        transformTobeMapped[i] = transformAftMapped[i];
      }
    }

    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>( ));
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>( ));
    pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>( ));

    pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
    pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
  }

  // 形成回环，根据优化后的位姿，更新所有cloudKeyPoses3D和cloudKeyPoses6D
  void correctPoses( )
  {
    if (aLoopIsClosed == true)
    {
      recentCornerCloudKeyFrames.clear( );
      recentSurfCloudKeyFrames.clear( );
      recentOutlierCloudKeyFrames.clear( );

      int numPoses = isamCurrentEstimate.size( );
      for (int i = 0; i < numPoses; ++i)
      {
        cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation( ).y( );
        cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation( ).z( );
        cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation( ).x( );

        cloudKeyPoses6D->points[i].x     = cloudKeyPoses3D->points[i].x;
        cloudKeyPoses6D->points[i].y     = cloudKeyPoses3D->points[i].y;
        cloudKeyPoses6D->points[i].z     = cloudKeyPoses3D->points[i].z;
        cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation( ).pitch( );
        cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation( ).yaw( );
        cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation( ).roll( );
      }
      aLoopIsClosed = false;
    }
  }

  void publishTF( )
  {
    if (isnan(transformAftMapped[0])) transformAftMapped[0] = 0.0;
    if (isnan(transformAftMapped[1])) transformAftMapped[1] = 0.0;
    if (isnan(transformAftMapped[2])) transformAftMapped[2] = 0.0;
    if (isnan(transformAftMapped[3])) transformAftMapped[3] = 0.0;
    if (isnan(transformAftMapped[4])) transformAftMapped[4] = 0.0;
    if (isnan(transformAftMapped[5])) transformAftMapped[5] = 0.0;

    if (isnan(transformBefMapped[0])) transformBefMapped[0] = 0.0;
    if (isnan(transformBefMapped[1])) transformBefMapped[1] = 0.0;
    if (isnan(transformBefMapped[2])) transformBefMapped[2] = 0.0;
    if (isnan(transformBefMapped[3])) transformBefMapped[3] = 0.0;
    if (isnan(transformBefMapped[4])) transformBefMapped[4] = 0.0;
    if (isnan(transformBefMapped[5])) transformBefMapped[5] = 0.0;

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

    odomAftMapped.header.stamp            = ros::Time( ).fromSec(timeLaserOdometry);
    odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
    odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
    odomAftMapped.pose.pose.orientation.z = geoQuat.x;
    odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    odomAftMapped.pose.pose.position.x    = transformAftMapped[3];
    odomAftMapped.pose.pose.position.y    = transformAftMapped[4];
    odomAftMapped.pose.pose.position.z    = transformAftMapped[5];
    // “twist”对应于机器人子坐标系（通常是移动基座坐标系）下的速度以及一个可选的速度估计协方差
    odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
    odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
    odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
    odomAftMapped.twist.twist.linear.x  = transformBefMapped[3];
    odomAftMapped.twist.twist.linear.y  = transformBefMapped[4];
    odomAftMapped.twist.twist.linear.z  = transformBefMapped[5];
    pubOdomAftMapped.publish(odomAftMapped);

    aftMappedTrans.stamp_ = ros::Time( ).fromSec(timeLaserOdometry);
    aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
    tfBroadcaster.sendTransform(aftMappedTrans);
  }

  void publishXYZTF( )
  {
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]);

    odomXYZAftMapped.header.frame_id = "/map";
    odomXYZAftMapped.child_frame_id  = "/aft_xyz_mapped";

    odomXYZAftMapped.header.stamp            = ros::Time( ).fromSec(timeLaserOdometry);
    odomXYZAftMapped.pose.pose.orientation.x = geoQuat.x;
    odomXYZAftMapped.pose.pose.orientation.y = geoQuat.y;
    odomXYZAftMapped.pose.pose.orientation.z = geoQuat.z;
    odomXYZAftMapped.pose.pose.orientation.w = geoQuat.w;
    odomXYZAftMapped.pose.pose.position.x    = transformAftMapped[5];
    odomXYZAftMapped.pose.pose.position.y    = transformAftMapped[3];
    odomXYZAftMapped.pose.pose.position.z    = transformAftMapped[4]; //-transformAftMapped[4]
    odomXYZAftMapped.twist.twist.angular.x   = transformBefMapped[2];
    odomXYZAftMapped.twist.twist.angular.y   = transformBefMapped[0];
    odomXYZAftMapped.twist.twist.angular.z   = transformBefMapped[1];
    odomXYZAftMapped.twist.twist.linear.x    = transformBefMapped[5];
    odomXYZAftMapped.twist.twist.linear.y    = transformBefMapped[3];
    odomXYZAftMapped.twist.twist.linear.z    = transformBefMapped[4]; //-transformBefMapped[4]
    pubOdomXYZAftMapped.publish(odomXYZAftMapped);

    aftMappedXYZTrans.stamp_ = ros::Time( ).fromSec(timeLaserOdometry);
    aftMappedXYZTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    aftMappedXYZTrans.setOrigin(tf::Vector3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])); //-transformAftMapped[4]
    tfXYZBroadcaster.sendTransform(aftMappedXYZTrans);
  }

  void publishXYTF( )
  {
    // std::cout << "publishXYTF "<< std::endl;

    odomXYZAftMapped.header.stamp          = ros::Time( ).fromSec(timeLaserOdometry);
    odomXYZAftMapped.pose.pose.position.x  = transformAftMapped[5];
    odomXYZAftMapped.pose.pose.position.y  = transformAftMapped[3];
    odomXYZAftMapped.pose.pose.position.z  = 0.0;
    odomXYZAftMapped.pose.pose.orientation = tf::createQuaternionMsgFromYaw(transformAftMapped[1]);
    odomXYZAftMapped.twist.twist.linear.x  = transformBefMapped[5]; // linear speed
    odomXYZAftMapped.twist.twist.linear.y  = transformBefMapped[3];
    odomXYZAftMapped.twist.twist.linear.z  = 0.0;
    odomXYZAftMapped.twist.twist.angular.z = transformBefMapped[1]; // angular speed
    // publish the message
    pubOdomXYZAftMapped.publish(odomXYZAftMapped);

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromYaw(transformAftMapped[1]);
    aftMappedXYZTrans.stamp_          = ros::Time( ).fromSec(timeLaserOdometry);
    aftMappedXYZTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    aftMappedXYZTrans.setOrigin(tf::Vector3(transformAftMapped[5], transformAftMapped[3], 0.0));

    //         aftMappedXYZTrans.transform.translation.x =
    //         transformAftMapped[5]; aftMappedXYZTrans.transform.translation.y
    //         = transformAftMapped[3];
    //         aftMappedXYZTrans.transform.translation.z = 0.0;
    //         aftMappedXYZTrans.transform.rotation =
    //         tf::createQuaternionMsgFromYaw(transformAftMapped[1]);
    tfXYZBroadcaster.sendTransform(aftMappedXYZTrans);
  }

  void publishKeyPosesAndFrames( )
  {
    if (pubKeyPoses.getNumSubscribers( ) != 0)
    {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
      cloudMsgTemp.header.stamp    = ros::Time( ).fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubKeyPoses.publish(cloudMsgTemp);
    }

    if (pubRecentKeyFrames.getNumSubscribers( ) != 0)
    {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp); // contain surf and outlier
      cloudMsgTemp.header.stamp    = ros::Time( ).fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubRecentKeyFrames.publish(cloudMsgTemp);
    }
  }

  void clearCloud( )
  {
    laserCloudCornerFromMap->clear( );
    laserCloudSurfFromMap->clear( );
    laserCloudCornerFromMapDS->clear( );
    laserCloudSurfFromMapDS->clear( );
  }
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "lego_loam");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

  parameter::readParameters(pnh);

  MappingHandler mappingHandler(nh, pnh); // 回调函数接受点云等转存成pcl并标记，接收位姿并标记

  // 实例化一个线程对象: thread(constructor/function, arguments)
  // to process loop Closure; 回环检测线程，对因子图优化，但是并不会直接对关键帧的位姿进行修改，而是在scan2map优化的线程中进行
  std::thread loopthread(&MappingHandler::loopClosureThread, &mappingHandler);
  // to visualize map; 根据关键帧位姿和点云，发布全局地图
  std::thread visualizeMapThread(&MappingHandler::visualizeGlobalMapThread, &mappingHandler);

  ros::Rate rate(200);
  while (ros::ok( ))
  {
    ros::spinOnce( );

    mappingHandler.run( ); // 每300ms进行一次scan2map匹配，判断是否增加关键帧并进行全局优化，判断回环检测线程是否有输出

    rate.sleep( );
  }

  // join 是让当前主线程(main function)等待所有的子线程执行完，才能退出
  loopthread.join( );
  visualizeMapThread.join( );

  return 0;
}
