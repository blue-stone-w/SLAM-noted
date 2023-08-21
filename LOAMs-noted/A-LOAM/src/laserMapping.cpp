/*
Description:
Author     : Wang Junpeng
date       :
*/

#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
// c++ 11的标准的线程库：std::thread。
#include <iostream>
#include <string>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

int frameCount = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

int laserCloudCenWidth = 10; // 使当前帧位姿处于地图中心的偏移量
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21; // 大局部地图的栅格数量
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;

const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; // 4851

// 把多个带有索引的小点云栅格放在一起构成局部地图。这个变量存储了需要的小点云栅格的索引号。
int laserCloudValidInd[125];
// 发布局部地图以可视化,构成该局部地图的小点云栅格的索引
int laserCloudSurroundInd[125];

// input: from odoms
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());

// ouput: all visualble cube points; 保存用于可视化的局部地图的点云
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());

// surround points in map to build tree || 用于优化的小局部地图（后文有详解）
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

// input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// points in every cube/栅格；将三维的局部栅格地图的栅格用一维数组表示
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

// kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};         // ceres求解得到的结果放在这个数组中，前四位为四元数，后三位为平移
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);  // 将相应的数据映射为四元数和平移
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4); // 这样当parameters变化时，q和t可以实时更新

// wmap_T_odom * odom_T_curr = wmap_T_curr; 地图坐标系和里程计坐标系之间的变换 * 里程计当前位姿 = 地图坐标系中的位姿
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0); // map's world frame
Eigen::Vector3d t_wmap_wodom(0, 0, 0);
Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0); // odom's world frame
Eigen::Vector3d t_wodom_curr(0, 0, 0);

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf; // 存储里程计的buffer
std::mutex mBuf;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

// 里程计坐标系下点的坐标; 地图坐标系下点的坐标
PointType pointOri, pointSel; // 同odometry中作为点处理函数的输入和输出

ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes;
ros::Publisher pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

nav_msgs::Path laserAfterMappedPath;

// set 后端优化的 initial guess
void transformAssociateToMap()
{
  // 世界坐标系下的位姿 = 地图坐标系和里程计坐标系的变换 * 里程计在世界坐标系下的位姿
  q_w_curr = q_wmap_wodom * q_wodom_curr;
  t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void transformUpdate()
{
  // 地图坐标系和里程计坐标系的变换 = 世界坐标系下的位姿 / 里程计在世界坐标系下的位姿
  q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
  t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
  // po->intensity = 1.0;
}

void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
  Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
  po->x = point_curr.x();
  po->y = point_curr.y();
  po->z = point_curr.z();
  po->intensity = pi->intensity;
}

// 回调函数将消息放入各自队列
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
  mBuf.lock(); // 此时其他线程不能操作该容器
  cornerLastBuf.push(laserCloudCornerLast2);
  mBuf.unlock();
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
  mBuf.lock();
  surfLastBuf.push(laserCloudSurfLast2);
  mBuf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
  mBuf.lock();
  fullResBuf.push(laserCloudFullRes2);
  mBuf.unlock();
}

// receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
  mBuf.lock();
  odometryBuf.push(laserOdometry);
  mBuf.unlock();

  // high frequence publish
  Eigen::Quaterniond q_wodom_curr;
  Eigen::Vector3d t_wodom_curr;
  q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x; // 把ros的消息转换为eigen的数据格式
  q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
  q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
  q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
  t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
  t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
  t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

  Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr; // 此处地图和里程计之间的转换已经优化
  Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.child_frame_id = "/aft_mapped";
  odomAftMapped.header.stamp = laserOdometry->header.stamp;
  odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
  odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
  odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
  odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
  odomAftMapped.pose.pose.position.x = t_w_curr.x();
  odomAftMapped.pose.pose.position.y = t_w_curr.y();
  odomAftMapped.pose.pose.position.z = t_w_curr.z();
  pubOdomAftMappedHighFrec.publish(odomAftMapped);
}

// 主处理线程
void process()
{
  while (1)
  {
    // 三种点和里程计都不为空
    while (!cornerLastBuf.empty() && !surfLastBuf.empty() && !fullResBuf.empty() && !odometryBuf.empty())
    {
      mBuf.lock();

      // 前端频率高于后端，需要去除一些点防止内存爆炸；
      // 因为后端耗时长，buf中的最新数据可能是很久之前的。去除部分数据使后端尽可能处理接近当前时间的数据，提高实时性。
      // 在后续代码中清理了corner buffer。此处再以cornerLastBuf为基准，把其他buffer中时间戳小的pop掉。
      while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
      {
        odometryBuf.pop();
      }
      if (odometryBuf.empty())
      {
        mBuf.unlock();
        break;
      }
      while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
      {
        surfLastBuf.pop();
      }
      if (surfLastBuf.empty())
      {
        mBuf.unlock();
        break;
      }
      while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
      {
        fullResBuf.pop();
      }
      if (fullResBuf.empty())
      {
        mBuf.unlock();
        break;
      }

      timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
      timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
      timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
      timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
      // 时间戳应当一致
      if (timeLaserCloudCornerLast != timeLaserOdometry ||
          timeLaserCloudSurfLast != timeLaserOdometry ||
          timeLaserCloudFullRes != timeLaserOdometry)
      {
        printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
        printf("unsync messeage!");
        mBuf.unlock();
        break;
      }

      // 点云转换为pcl的数据格式
      laserCloudCornerLast->clear();
      pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
      cornerLastBuf.pop();

      laserCloudSurfLast->clear();
      pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
      surfLastBuf.pop();

      laserCloudFullRes->clear();
      pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
      fullResBuf.pop();

      // lidar数据转换为eigen数据
      q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
      q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
      q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
      q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
      t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
      t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
      t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
      odometryBuf.pop();

      while (!cornerLastBuf.empty())
      { // 清理这个corner buffer
        cornerLastBuf.pop();
        printf("drop lidar frame in mapping for real time performance \n");
      }

      mBuf.unlock();
      TicToc t_whole;
      // 根据前端结果，得到后端的一个初始估计值

      transformAssociateToMap();

      // 栅格为边长为50m的立方体；21*21*11个栅格的局部地图，水平1公里，纵向500m；其余部分舍弃
      // 动态更新局部地图
      TicToc t_shift;
      // 根据初始估计值计算寻找当前位姿在地图中的索引，一个各自边长为50m
      // 后端的地图本质上是一个以当前点为中心的一个栅格地图
      int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;  // 确保四舍五入；
      int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight; // 通过这个偏移量使当前帧的位姿在地图的中心
      int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

      // C语言为向0取整，例如int（-1.66）为-1,但我们应该取-2,因此此处自减1
      if (t_w_curr.x() + 25.0 < 0) // 每个栅格的边长为50m，
      {
        centerCubeI--;
      }
      if (t_w_curr.y() + 25.0 < 0)
      {
        centerCubeJ--;
      }
      if (t_w_curr.z() + 25.0 < 0)
      {
        centerCubeK--;
      }

      // 如果当前栅格索引小于3,说明当前点已接近地图边界，需要将地图整体往X正方向移动
      while (centerCubeI < 3)
      {
        // i、j、k可视作该栅格在地图中的坐标
        for (int j = 0; j < laserCloudCenHeight; j++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int i = laserCloudWidth - 1;
            // 临时使用的用于地图移位的点云指针；从最大值开始移动；取出最右端点云
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            // 整体右移
            for (; i >= 1; i--)
            { // 将i-1的点云转移到i
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            }
            // 此时i=0,也就是最左边的栅格赋值了之前最右边的栅格
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
            // 该点云清零，由于是指针操作，相当于最左边的栅格清空
            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeI++;        // 索引右移
        laserCloudCenWidth++; // 偏移量
      }

      // 同理X如果抵达右边界，就整体左移
      while (centerCubeI >= laserCloudWidth - 3)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int i = 0;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            for (; i < laserCloudWidth - 1; i++)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }
        centerCubeI--;
        laserCloudCenWidth--;
      }

      while (centerCubeJ < 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int j = laserCloudHeight - 1;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            for (; j >= 1; j--)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }
        centerCubeJ++;
        laserCloudCenHeight++;
      }

      while (centerCubeJ >= laserCloudHeight - 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int k = 0; k < laserCloudDepth; k++)
          {
            int j = 0;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            for (; j < laserCloudHeight - 1; j++)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
      }

      while (centerCubeK < 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int j = 0; j < laserCloudHeight; j++)
          {
            int k = laserCloudDepth - 1;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            for (; k >= 1; k--)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeK++;
        laserCloudCenDepth++;
      }

      while (centerCubeK >= laserCloudDepth - 3)
      {
        for (int i = 0; i < laserCloudWidth; i++)
        {
          for (int j = 0; j < laserCloudHeight; j++)
          {
            int k = 0;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            for (; k < laserCloudDepth - 1; k++)
            {
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
          }
        }

        centerCubeK--;
        laserCloudCenDepth--;
      }
      // 此处完成局部栅格地图的更新，保证当前帧不会在局部地图的边缘，才可以从地图中获取足够的约束

      int laserCloudValidNum = 0;
      int laserCloudSurroundNum = 0;

      // 以当前栅格为中心，选出地图一定范围内的点云；此处为250*250*100的小局部地图，用于当前帧的配准
      for (int i = centerCubeI; i <= centerCubeI + 2; i++)
      {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
        {
          for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
          {
            // 上述局部地图的更新可以保证栅格索引不会越界；但谨慎起见，仍然判断栅格索引是否越界
            if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth)
            {
              // 记录各自的索引
              laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
              laserCloudValidNum++;
              laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
              laserCloudSurroundNum++;
            }
          }
        }
      }

      laserCloudCornerFromMap->clear(); // 点云指针；surround points in map to build tree
      laserCloudSurfFromMap->clear();
      // 开始构建优化这一帧的小局部地图
      for (int i = 0; i < laserCloudValidNum; i++)
      {
        *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]]; // 第i个有效栅格——>得到索引——>得到该栅格的点云
        *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
      }
      int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
      int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

      // 为减少运算量，对当前帧点云下采样
      pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>);
      downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
      downSizeFilterCorner.filter(*laserCloudCornerStack);
      int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

      pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
      downSizeFilterSurf.setInputCloud(laserCloudSurfLast); // 下采样的原始点云
      downSizeFilterSurf.filter(*laserCloudSurfStack);      // 保存下采样结果
      int laserCloudSurfStackNum = laserCloudSurfStack->points.size(pointSel);

      printf("map prepare time %f ms\n", t_shift.toc());
      printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
      // 最终有效点云数目进行判断。可参考的帧太少，影响优化效果
      if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
      {
        TicToc t_opt;
        TicToc t_tree;
        // 将局部小地图放入kd树，用kd树最近邻搜索
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
        printf("-----------------------------map-------------------------\n");
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
        printf("build tree time %f ms \n", t_tree.toc());

        for (int iterCount = 0; iterCount < 2; iterCount++)
        { // 建立对应关系的迭代次数不超过2次
          // ceres::LossFunction *loss_function = NULL;
          // 参考前端里程计的代码注释
          // 建立ceres问题
          // 核函数，避免oputlier
          ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); // 残差大于0.1时降低权重，小于0.1不作权重更改
          ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
          ceres::Problem::Options problem_options;

          ceres::Problem problem(problem_options);
          problem.AddParameterBlock(parameters, 4, q_parameterization); // 待优化的变量：旋转的指针位置，4字节，
          problem.AddParameterBlock(parameters + 4, 3);                 // 待优化变量：平移的指针位置（从第4个数字开始为平移的数据），3字节

          TicToc t_data;
          int corner_num = 0;
          // 构建角点相关的约束
          for (int i = 0; i < laserCloudCornerStackNum; i++)
          {                                              // 遍历当前帧所有点（已经下采样）
            pointOri = laserCloudCornerStack->points[i]; // 数据类型为点；
            // double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
            // 里程计坐标系下的坐标转换为地图坐标系下的坐标；转换参数为当前点和两个坐标系的位姿变换的初始值
            pointAssociateToMap(&pointOri, &pointSel);
            // 在小局部地图中寻找最近的5个点
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0)
            { // 5个最近点中，最远的距离不得超过1米，否则为无效约束
              std::vector<Eigen::Vector3d> nearCorners;
              Eigen::Vector3d center(0, 0, 0);
              for (int j = 0; j < 5; j++)
              {
                Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
              }
              center = center / 5.0; // 5个点的均值

              // 需要保证5个点共线；无线束ID等信息，必须通过其它方式判断; 如果有行号和列号，可以辅助判断是否共线
              // 若5个点在同一直线上，则5个点的协方差矩阵的特征值为一大两小；最大特征值对应的特征向量为直线的方向向量
              Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
              for (int j = 0; j < 5; j++)
              { // 构建协方差矩阵
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose(); // 3维向量变为3*3矩阵
              }
              // 特征值分解；//用covMat初始化一个对象，包含特征值和特征向量
              Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

              // if is indeed line feature
              // note Eigen library sort eigenvalues in increasing order;
              Eigen::Vector3d unit_direction = saes.eigenvectors().col(2); // 升序排列特征值，第3个特征向量为线特征的方向
              Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
              {                                         // 最大的特征值远大于另外两个，说明5点共线
                Eigen::Vector3d point_on_line = center; // 一个点+方向=一条直线
                Eigen::Vector3d point_a, point_b;
                // 根据拟合的直线，构建两个虚拟点
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;
                // 构建约束，约束和lidar odom保持一致
                // 参数：当前点，直线上的两点，时间比例
                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                corner_num++;
              }
            }
            /*
            else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
            {
              Eigen::Vector3d center(0, 0, 0);
              for (int j = 0; j < 5; j++)
              {
                Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                          laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                          laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
              }
              center = center / 5.0;
              Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
              problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
            }
            */
          }

          int surf_num = 0;
          // 此处没有使用特征值判断平面的拟合质量
          for (int i = 0; i < laserCloudSurfStackNum; i++)
          {
            pointOri = laserCloudSurfStack->points[i];
            // double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            // 平面方程： 1、Ax+By+Cz+D=0；2、Ax+By+Cz+1=0
            // 三个未知数（ABC），五个方程（五个点）
            // 五个方程对应的未知数的系数的矩阵（使用方程2）
            Eigen::Matrix<double, 5, 3> matA0;
            // 五个方程对应的右侧常数项(使用方程2）
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < 1.0)
            {
              for (int j = 0; j < 5; j++)
              {
                matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
              }
              // find the norm of plane 平面单位法向量
              // 使用eigen接口求解该方程，解为该平面的法向量（Ax=B）；使用qr分解
              Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0); // 结果为方程2中的（A，B，C）
              double negative_OA_dot_norm = 1 / norm.norm();                   // 法向量的模的倒数
              norm.normalize();                                                // 法向量的归一化

              // Here n(pa, pb, pc) is unit norm of plane
              bool planeValid = true;
              for (int j = 0; j < 5; j++)
              {
                // if OX * n > 0.2, then plane is not fit well
                // 点到平面的距离公式:d=|Ax+By+Cz+1|/|n|;A/|n|=n(0);
                // 点到拟合的平面的距离不能大于阈值
                if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                         norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                         norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z +
                         negative_OA_dot_norm) > 0.2)
                {
                  planeValid = false;
                  break;
                }
              }
              Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              if (planeValid)
              {
                // 利用平面方程构建约束，和前端构建形式不同(使用了函数指针,一个只想函数的指针)
                ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                surf_num++;
              }
            }
            /*
            else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
            {
              Eigen::Vector3d center(0, 0, 0);
              for (int j = 0; j < 5; j++)
              {
                Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
                          laserCloudSurfFromMap->points[pointSearchInd[j]].y,
                          laserCloudSurfFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
              }
              center = center / 5.0;
              Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
              problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
            }
            */
          }
          // printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
          // printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

          printf("mapping data assosiation time %f ms \n", t_data.toc());

          // 参照里程计注释
          // 调用ceres求解
          TicToc t_solver;
          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR; // 稠密
          options.max_num_iterations = 4;
          options.minimizer_progress_to_stdout = false;
          options.check_gradients = false;
          options.gradient_check_relative_precision = 1e-4;
          ceres::Solver::Summary summary; // 求解的结果报告，如残差为多大
          ceres::Solve(options, &problem, &summary);
          printf("mapping solver time %f ms \n", t_solver.toc()); // 求解时间

          // printf("time %f \n", timeLaserOdometry);
          // printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
          // printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
          //	   parameters[4], parameters[5], parameters[6]);
        }
        printf("mapping optimization time %f \n", t_opt.toc()); // 优化两次的时间
      }
      else
      {
        ROS_WARN("time Map corner and surf num are not enough");
      }
      transformUpdate();

      TicToc t_add;
      // 把优化后的当前帧角点加到局部地图中去
      for (int i = 0; i < laserCloudCornerStackNum; i++)
      {
        // 该点根据位姿投影到地图坐标系
        pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
        // 算出这个点所在栅格的索引
        int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
        // 因为向0取整，所以同上
        if (pointSel.x + 25.0 < 0)
          cubeI--;
        if (pointSel.y + 25.0 < 0)
          cubeJ--;
        if (pointSel.z + 25.0 < 0)
          cubeK--;
        // 如果越界就不更新地图
        if (cubeI >= 0 && cubeI < laserCloudWidth &&
            cubeJ >= 0 && cubeJ < laserCloudHeight &&
            cubeK >= 0 && cubeK < laserCloudDepth)
        {
          // 根据xyz的索引计算在一维数组中的索引
          int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
          laserCloudCornerArray[cubeInd]->push_back(pointSel); // 把当前点放入
        }
      }

      for (int i = 0; i < laserCloudSurfStackNum; i++)
      {
        pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

        int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

        if (pointSel.x + 25.0 < 0)
          cubeI--;
        if (pointSel.y + 25.0 < 0)
          cubeJ--;
        if (pointSel.z + 25.0 < 0)
          cubeK--;

        if (cubeI >= 0 && cubeI < laserCloudWidth &&
            cubeJ >= 0 && cubeJ < laserCloudHeight &&
            cubeK >= 0 && cubeK < laserCloudDepth)
        {
          int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
          laserCloudSurfArray[cubeInd]->push_back(pointSel);
        }
      }
      printf("add points time %f ms\n", t_add.toc());

      TicToc t_filter;
      // 对当前帧涉及到的小局部地图的栅格下采样，而不是所有局部地图的栅格
      // 线和面过于集中，从kd树中提取的点过于集中，不利于优化
      for (int i = 0; i < laserCloudValidNum; i++)
      {
        int ind = laserCloudValidInd[i];

        pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
        downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
        downSizeFilterCorner.filter(*tmpCorner);
        laserCloudCornerArray[ind] = tmpCorner;

        pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
        downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
        downSizeFilterSurf.filter(*tmpSurf);
        laserCloudSurfArray[ind] = tmpSurf;
      }
      printf("filter time %f ms \n", t_filter.toc());

      TicToc t_pub;
      // publish surround map for every 5 frame
      if (frameCount % 5 == 0)
      {
        laserCloudSurround->clear();
        // 发布该当前帧相关的局部地图
        for (int i = 0; i < laserCloudSurroundNum; i++)
        {
          int ind = laserCloudSurroundInd[i];
          // 用于可视化，不需要区分线点和面点
          *laserCloudSurround += *laserCloudCornerArray[ind];
          *laserCloudSurround += *laserCloudSurfArray[ind];
        }
        sensor_msgs::PointCloud2 laserCloudSurround3;
        pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);                   // pcl转换为ros格式
        laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry); // 时间戳
        laserCloudSurround3.header.frame_id = "/camera_init";                      // 坐标系
        pubLaserCloudSurround.publish(laserCloudSurround3);                        // 发布
      }
      // 每隔20帧发布一次大局部地图
      if (frameCount % 20 == 0)
      {
        printf("旋转：%f 平移:%f\n", q_w_curr.x(), t_w_curr.x());
        pcl::PointCloud<PointType> laserCloudMap;
        for (int i = 0; i < 4851; i++)
        { // 大局部地图的栅格总数
          laserCloudMap += *laserCloudCornerArray[i];
          laserCloudMap += *laserCloudSurfArray[i];
        }
        sensor_msgs::PointCloud2 laserCloudMsg;
        pcl::toROSMsg(laserCloudMap, laserCloudMsg);
        laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudMsg.header.frame_id = "/camera_init";
        pubLaserCloudMap.publish(laserCloudMsg);
      }

      int laserCloudFullResNum = laserCloudFullRes->points.size();
      for (int i = 0; i < laserCloudFullResNum; i++)
      {
        pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
      }

      sensor_msgs::PointCloud2 laserCloudFullRes3;
      pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
      laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      laserCloudFullRes3.header.frame_id = "/camera_init";
      pubLaserCloudFullRes.publish(laserCloudFullRes3);

      printf("mapping pub time %f ms \n", t_pub.toc());

      printf("whole mapping time %f ms +++++\n", t_whole.toc());

      // 发布当前位姿
      nav_msgs::Odometry odomAftMapped;
      odomAftMapped.header.frame_id = "/camera_init";
      odomAftMapped.child_frame_id = "/aft_mapped";
      odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
      odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
      odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
      odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
      odomAftMapped.pose.pose.position.x = t_w_curr.x();
      odomAftMapped.pose.pose.position.y = t_w_curr.y();
      odomAftMapped.pose.pose.position.z = t_w_curr.z();
      pubOdomAftMapped.publish(odomAftMapped);

      // 发布轨迹，用于rviz可视化
      geometry_msgs::PoseStamped laserAfterMappedPose;
      laserAfterMappedPose.header = odomAftMapped.header;
      laserAfterMappedPose.pose = odomAftMapped.pose.pose;
      laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
      laserAfterMappedPath.header.frame_id = "/camera_init";
      laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
      pubLaserAfterMappedPath.publish(laserAfterMappedPath);

      // 发布tf
      static tf::TransformBroadcaster br;
      tf::Transform transform;                                                 // 里程计和地图之间的位姿变换
      tf::Quaternion q;                                                        // 里程计和地图之间的旋转
      transform.setOrigin(tf::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2))); // 平移
      q.setW(q_w_curr.w());
      q.setX(q_w_curr.x());
      q.setY(q_w_curr.y());
      q.setZ(q_w_curr.z());
      transform.setRotation(q);
      //
      br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

      frameCount++;
    }
    std::chrono::milliseconds dura(2); // 2毫秒
    std::this_thread::sleep_for(dura); // 暂停2毫秒
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  float lineRes = 0;
  float planeRes = 0;
  // 从参数服务器获取参数；参数可能写在launch文件中
  nh.param<float>("mapping_line_resolution", lineRes, 0.4); // 地图中的特征的分辨率
  nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
  printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
  downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes); // 体素滤波器的边长
  downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

  ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);
  ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);
  // 前端里程计位姿
  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);
  // 全部点云
  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);

  // 发布局部地图
  pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
  pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);
  pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

  for (int i = 0; i < laserCloudNum; i++)
  {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>()); // 创建一个空指针
    laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
  }
  std::thread mapping_process{process}; // 创建了一个线程，线程名为“mapping_process”，入口函数为“process”

  ros::spin();

  return 0;
}