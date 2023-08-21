/*
Description:
Author     : Wang Junpeng
data       :
*/

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
// 互斥锁（英語：Mutual exclusion，缩写Mutex）:
// 用于多线程编程中，防止两条线程同时对同一公共资源（比如全局变量）进行读写的机制。
#include <queue>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0 // 0说明点云不需要去畸变

int corner_correspondence = 0, plane_correspondence = 0; // 角点和平面点的匹配数量

// constexpr表示该变量在编译期就可以算出来（前提是为了算出它所依赖的东西也是在编译期可以算出来的）
constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25; // 距离_sqrt
constexpr double NEARBY_SCAN = 2.5;          // 线数ID的差大于该数说明不是相邻scan

int skipFrameNum = 5; // 每隔skip帧就向后端输出一次该帧的特征点；如果算力不足可降频
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>()); // 全部点云

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1}; // 保存四元数
double para_t[3] = {0, 0, 0};    // 平移

// Eigen::Map 的作用是将一个已有的 C 数组映射为一个 Eigen 的向量或者矩阵。它的优点是：
// 可以使用 Eigen 向量和矩阵的各种操作函数；不会额外分配空间。即，它并非拷贝，而是依然使用已有数组的空间。
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q); // 将数组para_q映射为四元数q_last_curr
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);    // 将数组para_t映射为向量t_last_curr

// 队列的每个元素为从一帧点云中提取的一个相应点云
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;

// https://www.cnblogs.com/haippy/p/3237213.html#:~:text=std%3A%3Amutex%20%E4%BB%8B%E7%BB%8D&text=std%3A%3Amutex%20%E6%98%AFC,%E6%96%A5%E9%87%8F%E5%AF%B9%E8%B1%A1%E4%B8%8A%E9%94%81%E3%80%82
std::mutex mBuf; // 在该线程内起到互斥锁的作用

// undistort lidar point，把每个点投影到该帧的起始时刻
void TrasnsformToStart(PointType const *const pi, PointType *const po)
{                 // pi为输入点，po为输出点
  double s;       // interpolation ratio
  if (DISTORTION) // kitti已经补偿过了，因此不需要去畸变
  // intensity：整数部分是scan的索引，小数部分是相对起始时刻的时间
  {
    s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD; // int取整
  }
  else
  {
    s = 1.0; // 说明全部点都补偿到起始时刻
  }
  // 所有点都从结束时刻补偿到起始时刻
  // 匀速模型假设，速度变化不剧烈如汽车；手持设备不合适
  Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr); // s为插值比例；q_last_curr上一帧到当前帧的四元数
  Eigen::Vector3d t_point_last = s * t_last_curr;                                         // 同上
  Eigen::Vector3d point(pi->x, pi->y, pi->z);
  Eigen::Vector3d un_point = q_point_last * point + t_point_last; // 畸变补偿后的点；点的变换方程：P1=R*P2+t

  po->x = un_point.x();
  po->y = un_point.y();
  po->z = un_point.z();
  po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame
void TransformToEnd(PointType const *const pi, PointType *const po)
{
  // undistort point first
  pcl::PointXYZI un_point_tmp;
  TrasnsformToStart(pi, &un_point_tmp); // pi转换到pi所在帧的起始时刻得到的点放入un_point_tmp

  Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
  Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr); // 同上述的点的变换方程，过程相反
  po->x = point_end.x();
  po->y = point_end.y();
  po->z = point_end.z();

  // Remove distortion time info
  po->intensity = int(pi->intensity); // 取整；小数部分是相对起始时刻的时间，已经完成畸变补偿，不再需要
}

// 接收四种特征点，送去各自的队列
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
  mBuf.lock();
  cornerSharpBuf.push(cornerPointsSharp2); // 把新的点云加入buf的后面
  mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
  mBuf.lock();
  cornerLessSharpBuf.push(cornerPointsLessSharp2);
  mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
  mBuf.lock();
  surfFlatBuf.push(surfPointsFlat2);
  mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
  mBuf.lock();
  surfLessFlatBuf.push(surfPointsLessFlat2);
  mBuf.unlock();
}

// receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
  mBuf.lock();
  fullPointsBuf.push(laserCloudFullRes2);
  mBuf.unlock();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  nh.param<int>("mapping_skip_frame", skipFrameNum, 2); // 下采样的频率
  printf("LO mapping %d Hz \n", 10 / skipFrameNum);

  /// printf("no sub&pub\n");
  // 订阅四种点和所有点；每个消息为一个点云
  ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler); // 话题，消息队列的长度，回调函数
  ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
  ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
  ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);
  /// printf("subed\n");
  // 发布两种点、全部点
  ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
  ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
  ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
  ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
  ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100); // 里程计的轨迹
  /// printf("pubed\n");
  nav_msgs::Path laserPath; // 一个存储每帧位姿的向量

  int frameCount = 0; // 计算帧的数量
  ros::Rate rate(100);

  while (ros::ok())
  {                  // 没有ctrl+c取消，则持续执行
    ros::spinOnce(); // 触发一次回调

    /// std::cin.get();
    printf("---1---\n");
    // 5种点都不为空；5个消息都有
    if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
        !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty())
    {
      // front为队列的第一个；header为头类，包含标识信息seq(0)、stamp()、frame_id()；stamp为时间
      timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec(); // front为首个元素；
      timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
      timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec(); // toSec将Time转为double型时间
      timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
      timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

      // 比较buf中第一个点的时间；因为同一帧时间相同，因此这里比较是否为同一帧
      if (timeCornerPointsSharp != timeLaserCloudFullRes ||
          timeCornerPointsLessSharp != timeLaserCloudFullRes ||
          timeSurfPointsFlat != timeLaserCloudFullRes ||
          timeSurfPointsLessFlat != timeLaserCloudFullRes)
      {
        printf("unsync messeage!");
        ROS_BREAK();
      }
      printf("---3---\n");
      // 将点云消息转换成pcl格式
      mBuf.lock();
      // 取出buf中第一个相应点云
      cornerPointsSharp->clear(); // 清空该点
      pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
      cornerSharpBuf.pop(); // 删除第一个数据

      cornerPointsLessSharp->clear();
      printf("chw:%d %d\n", cornerLessSharpBuf.front()->height, cornerLessSharpBuf.front()->width);
      pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
      cornerLessSharpBuf.pop();
      printf("c:%d\n", cornerPointsLessSharp->points.size());

      surfPointsFlat->clear();
      pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
      surfFlatBuf.pop();

      surfPointsLessFlat->clear();
      printf("empty:%d\n", surfLessFlatBuf.empty());
      printf("shw:%d %d\n", surfLessFlatBuf.front()->height, surfLessFlatBuf.front()->width);
      pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
      printf("s:%d\n", surfPointsLessFlat->points.size());
      surfLessFlatBuf.pop();

      laserCloudFullRes->clear();
      pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
      fullPointsBuf.pop();
      mBuf.unlock();
      printf("---5---\n");

      TicToc t_whole;
      // initializing；一个什么都不做的初始化
      if (!systemInited)
      {
        systemInited = true;
        std::cout << "Initialization finished \n";
      }
      else
      {
        // 一帧中两种特征点的数量；突出的特征
        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        int surfPointsFlatNum = surfPointsFlat->points.size();
        printf("---7---\n");

        TicToc t_opt;
        // 进行俩次迭代
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
        {
          corner_correspondence = 0;
          plane_correspondence = 0;

          // ceres::LossFunction *loss_function = NULL;
          // 定义ceres的核函数;核：函数残差越大，在使用时的权重越小，可以有效避免个别点对结果的影响
          ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); // 残差大于0.1时降低权重，小于0.1不作权重更改
          // 由于旋转不满足一般意义的加法，因此这里使用ceres自带的local param类，用于处理四元数相加
          ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
          ceres::Problem::Options problem_options; // 问题的选项
          ceres::Problem problem(problem_options); // 使用问题的选项构建问题
          // 待优化变量是帧间位姿（平移和旋转），这里旋转用四元数表示
          problem.AddParameterBlock(para_q, 4, q_parameterization); //(double数组/指针，数组的长度，四元数加法)
          problem.AddParameterBlock(para_t, 3);
          printf("---9---\n");

          // 为每个点找寻最近点时，将该点补偿畸变后临时存放于该变量（point selected）
          pcl::PointXYZI pointSel;
          // 每个点的最近点的索引
          std::vector<int> pointSearchInd;
          // 每个点的最近点的距离
          std::vector<float> pointSearchSqDis;

          TicToc t_data;
          // find correspondence for corner features
          // 寻找角点的约束
          for (int i = 0; i < cornerPointsSharpNum; i++)
          {
            TrasnsformToStart(&(cornerPointsSharp->points[i]), &pointSel); // 运动补偿
            // 在上一帧所有角点构成的kd树中寻找距离当前帧最近的一个点
            // 当前转到起始时刻的点，找1个点，在上一帧kd数的点的ID，两点的距离；后面两个变量为向量
            kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1; // 此处为初始值，表示无效valid

            // 因为两帧之间的距离应该很短，因此两帧间的距离只有小于给定距离阈值才认为是有效约束,可以用于匹配
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {                                      // 最近的点小于阈值
              closestPointInd = pointSearchInd[0]; // 对应的最de近距离的点索引；仅有1个点，所以为[0]
              // 找到其所在线束id，线束信息在intensity的整数部分
              int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

              double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;

              // search in the direction of increasing scan line，即向上找
              for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
              {
                // if in the same scan line, continue
                // scan为一水平行，特征点是在一行内找到的，这意味着找到的线特征倾向于竖直方向，而非水平方向
                // 如果在同一根线束，两个点构成的直线是水平的，这条直线不可能是之前找到的线特征
                // 而匹配时计算的是点到相应的线特征的距离
                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                  continue; // 如果在同一根线束，跳过该点

                // if not in nearby scans，说明向上找没有相邻scan, end the loop
                if (int(laserCloudCornerLast->points[j].intensity) > closestPointScanID + NEARBY_SCAN)
                  break;

                // 上一帧的点 - 最新的点 的 距离
                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                if (pointSqDis < minPointSqDis2)
                {                              // find nearer point
                  minPointSqDis2 = pointSqDis; // 保存当前最小距离
                  minPointInd2 = j;            // 保存上一帧的点ind索引
                }
              }

              // search in the direction of decreasing scan line
              for (int j = closestPointInd - 1; j >= 0; --j)
              {
                // if in the same scan line, continue
                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                  continue;

                // if not in nearby scans, end the loop
                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                  break;

                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                if (pointSqDis < minPointSqDis2)
                { // find nearer point
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
            if (minPointInd2 >= 0)
            { // both closestPointInd and minPointInd2 is valid
              Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                         cornerPointsSharp->points[i].y,
                                         cornerPointsSharp->points[i].z);
              Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                           laserCloudCornerLast->points[closestPointInd].y,
                                           laserCloudCornerLast->points[closestPointInd].z);
              Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                           laserCloudCornerLast->points[minPointInd2].y,
                                           laserCloudCornerLast->points[minPointInd2].z);

              double s;
              if (DISTORTION)
                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
              else
                s = 1.0;
              ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s); // s为当前时间比例
              problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);                                  //(残差项(不可或缺的)，核函数，优化变量的指针/首地址，优化变量的指针)
              corner_correspondence++;
            }
          }
          printf("---11---\n");

          // find correspondence for plane features
          for (int i = 0; i < surfPointsFlatNum; ++i)
          {
            TrasnsformToStart(&(surfPointsFlat->points[i]), &pointSel);
            kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            printf("---12.0---\n");

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
              closestPointInd = pointSearchInd[0];
              // get closest point's scan ID
              int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
              double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
              printf("---12.1---\n");

              // search in the direction of increasing scan line
              // 确保三个点不会水平（三个点不在同一scan）或竖直（三个点两两不共scan时可能）地共线，这样才能构成面
              for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
              {
                // if not in nearby scans, end the loop
                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                  break;

                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                // if in the same or lower scan line
                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                {
                  // if in the higher scan line
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
              printf("---12.3---\n");

              // search in the direction of decreasing scan line
              for (int j = closestPointInd - 1; j >= 0; --j)
              {
                // if not in nearby scans, end the loop
                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                  break;

                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                // if in the same or higher scan line
                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                {
                  // find nearer point
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }

              if (minPointInd2 >= 0 && minPointInd3 >= 0)
              { // 三个点都有效
                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                           surfPointsFlat->points[i].y,
                                           surfPointsFlat->points[i].z);
                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                             laserCloudSurfLast->points[closestPointInd].y,
                                             laserCloudSurfLast->points[closestPointInd].z);
                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                             laserCloudSurfLast->points[minPointInd2].y,
                                             laserCloudSurfLast->points[minPointInd2].z);
                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                             laserCloudSurfLast->points[minPointInd3].y,
                                             laserCloudSurfLast->points[minPointInd3].z);

                double s;
                if (DISTORTION)
                  s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                else
                  s = 1.0;
                printf("---12.5---\n");
                // 选择的Cost Function为自动求导
                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                //(cost function, 核函数, 两个待优化参数)
                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                plane_correspondence++;
              }
            }
          }
          printf("---13---\n");
          // printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
          printf("data association time %f ms \n", t_data.toc());

          if ((corner_correspondence + plane_correspondence) < 10)
          {
            printf("less correspondence! *************************************************\n");
          }

          TicToc t_solver;
          ceres::Solver::Options options;               // 求解器的选项
          options.linear_solver_type = ceres::DENSE_QR; // 稠密矩阵的求解方法；因为此处的矩阵为稠密矩阵
          options.max_num_iterations = 4;
          options.minimizer_progress_to_stdout = false; // 不使用stdout
          ceres::Solver::Summary summary;               // 求解的结果报告，如残差为多大
          ceres::Solve(options, &problem, &summary);    //(求解的选项，问题，结果报告)
          printf("solver time %f ms \n", t_solver.toc());
          // 第一次优化后重新建立匹配对，再次优化
        }

        printf("optimization twice time %f \n", t_opt.toc());

        t_w_curr = t_w_curr + q_w_curr * t_last_curr; // 点的变换方程解耦
        q_w_curr = q_w_curr * q_last_curr;            // 里程计位姿：上一帧位姿 + 位姿变换 = 当前位姿
      }

      TicToc t_pub;

      // publish odometry，发布里程计结果
      nav_msgs::Odometry laserOdometry;
      laserOdometry.header.frame_id = "/camera_init";
      laserOdometry.child_frame_id = "/laser_odom";
      laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
      // 以四元数和平移向量发出
      laserOdometry.pose.pose.orientation.x = q_w_curr.x();
      laserOdometry.pose.pose.orientation.y = q_w_curr.y();
      laserOdometry.pose.pose.orientation.z = q_w_curr.z();
      laserOdometry.pose.pose.orientation.w = q_w_curr.w();
      laserOdometry.pose.pose.position.x = t_w_curr.x();
      laserOdometry.pose.pose.position.y = t_w_curr.y();
      laserOdometry.pose.pose.position.z = t_w_curr.z();
      pubLaserOdometry.publish(laserOdometry);

      geometry_msgs::PoseStamped laserPose;
      laserPose.header = laserOdometry.header;
      laserPose.pose = laserOdometry.pose.pose;
      laserPath.header.stamp = laserOdometry.header.stamp;
      laserPath.poses.push_back(laserPose);
      laserPath.header.frame_id = "/camera_init";
      pubLaserPath.publish(laserPath); // 传给ROS使用，一般用于rviz显示

      // transform corner features and plane features to the scan end point
      if (0)
      { // 可使用并行计算加速
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

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++)
        {
          TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }
      }

      // 将当前帧的点改为上一帧，用上一帧的点覆盖当前帧，用覆盖代替clear
      printf("1 c:%d--s:%d\n", laserCloudCornerLast->points.size(), laserCloudSurfLast->points.size());
      printf("2 c:%d--s:%d\n", cornerPointsLessSharp->points.size(), surfPointsLessFlat->points.size());
      pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
      cornerPointsLessSharp = laserCloudCornerLast;
      laserCloudCornerLast = laserCloudTemp;

      laserCloudTemp = surfPointsLessFlat;
      surfPointsLessFlat = laserCloudSurfLast;
      laserCloudSurfLast = laserCloudTemp;

      laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      laserCloudSurfLastNum = laserCloudSurfLast->points.size();

      // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

      // kd树设置为当前帧，用于下一帧的lidar odom使用
      printf("--------------------------odom1---------------------------\n");
      kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
      printf("--------------------------odom3---------------------------\n");
      kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
      printf("--------------------------odom5---------------------------\n");

      if (frameCount % skipFrameNum == 0)
      { // 每隔skip帧就向后端输出一次该帧的特征点；如果算力不足可降频
        frameCount = 0;

        // 后端为低频高精度，因此需要全部点
        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        // 该帧的各种点云的时间都是相同的，因此可以用一个时间赋值
        laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudCornerLast2.header.frame_id = "/camera";
        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudSurfLast2.header.frame_id = "/camera";
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudFullRes3.header.frame_id = "/camera";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);
      }
      printf("publication time %f ms \n", t_pub.toc());
      printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
      if (t_whole.toc() > 100)
        ROS_WARN("odometry process over 100ms"); // 处理时间大于帧之间的时间，可能会导致掉帧
      frameCount++;
    } // 判断点云非空后的处理完成
    rate.sleep();
  }
  return 0;
}