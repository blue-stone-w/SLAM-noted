/*
Description:
Author     : Wang Junpeng
date       :
*/

#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h> // 如果你使用的是noetic自带的OpenCV，需要改为"opencv2/core.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>        //点云类的具体定义可参考文件<pcl/point_cloud.h>；
#include <pcl/point_types.h>        //点类的具体定义
#include <pcl/filters/voxel_grid.h> //体素滤波器
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

// 首先定义全局变量
const double scanPeriod = 0.1;   // 扫描周期
const int systemDelay = 0;       // 初始化系统需要的帧的数量
int systemInitCount = 0;         // 已经用于系统初始化的帧的数量
bool systemInited = false;       // 系统是否已经初始化
int N_SCANS = 0;                 // 线束的数量，velodyne雷达有16、32、64三种，在一个竖直扫描中的点的个数；
float cloudCurvature[400000];    // 保存每个点的曲率
int cloudSortInd[400000];        // 该数组为该点的原始位置，即该点的索引;会有排序操作使点按照曲率从小到大排序
int cloudNeighborPicked[400000]; // 标志位，1说明该点应避免选为特征点，或者已经处理过，不能再次选为特征点
int cloudLabel[400000];          // 保存每个点的标签

bool comp(int i, int j) { return (cloudCurvature[i]) < cloudCurvature[j]; } // 用于判断大小的函数，返回值为布尔值

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan; // 创建一个向量，元素类型为ros::Publisher

bool PUB_EACH_LINE = false; // 发布点云的方式，是否按照每个scan发布

double MINIMUM_RANGE = 0.1; // 小于此距离的点为无效点，需要剔除

template <typename PointT> // template function
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out,
                            float thres)
{
  if (&cloud_in != &cloud_out)
  {
    // 判断输入参数和输出参数是否为同一个东西。如果不是一个东西，则为输出参数声明一个空间
    // 若是相同的东西，则不再声明空间，可以节约空间
    cloud_out.header = cloud_in.header; // header部分应该相同
    // 为cloud_out设置合适的大小，以便可以储存从clout_in中筛选出来的符合要求的点
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0; // 用于计数cloud_out的大小

  // 大于阈值thres的点放入cloud_out
  for (size_t i = 0; i < cloud_in.points.size(); ++i)
  {
    if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size())
  { // 如果存在剔除的点，重新设置cloud_out的大小
    cloud_out.points.resize(j);
  }

  // 把点云变为无序点云，高度为1，宽度为所有值
  cloud_out.height = 1;                       // ros的有序点云消息中，高度为线数
  cloud_out.width = static_cast<uint32_t>(j); // 在不检查类型以保证转换安全性的情况下将j转换为uint32类型
  cloud_out.is_dense = true;                  // 点云为稠密点，后续应该使用稠密矩阵的求解方式
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  // 如果系统没有初始化，那么前几帧的数据质量不好。因此舍弃前几帧至系统达到稳定；
  //  kitti数据集质量好，因此可以将延迟（初始化系统需要的帧的数量）设置为0
  if (!systemInited)
  {
    systemInitCount++;
    if (systemInitCount >= systemDelay)
    {
      systemInited = true;
    }
    else
    {
      return; // 不再继续处理该帧点云，即舍弃该帧点云
    }
  }

  //*****激光雷达性质*****
  TicToc t_whole; // TicToc类的对象在创建时，已经根据构造函数得到创建时刻的时间值
  TicToc t_prepare;
  std::vector<int> scanStartInd(N_SCANS, 0); // 创建向量;Vector<类型>标识符(最大容量,初始所有值)
  std::vector<int> scanEndInd(N_SCANS, 0);   // 一个扫描周期，起始和终止的序号；起始和终止的最大容量为线数

  pcl::PointCloud<pcl::PointXYZ> laserCloudIn; // 创建点云变量，点的类型为PointXYZ;输入点云
  // 从laserCloudMsg消息中读取数据存入laserCloudIn；把点云从ros格式转为pcl格式
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  std::vector<int> indices; // 用于保存去除无效点后的点的索引
  // 不是每个激光点都能够返回并被接收，例如方向为天空的点。
  // 这些激光点就是nan点（可以视为不存在的点，但在原始的点云数据中却占据位置），需要去除
  // index：the mapping (ordered): cloud_out[i] = cloud_in[index[i]]
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

  int cloudSize = laserCloudIn.points.size(); // 输入点云的大小
  // 计算起始点和结束点的角度。坐标系中逆时针为正方向。由于激光雷达是顺时针旋转，这里取反相当于转成了逆时针。
  // 点云第一个点的方向orientation
  float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
  // 点云最后一个点的方向orientation。atan2的范围为[-pi, pi]。
  // 由于三角函数的周期性，结束点的角度应该加上2*pi表示旋转了一周，这样才符合实际情况
  float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

  // 出现例外情况，差值小于pi或大于3pi，做调整以使处于合理范围pi~3pi（2pi附近）
  // 例如起始点为179，结束点为-179；或相反
  if (endOri - startOri > 3 * M_PI)
  {
    endOri -= 2 * M_PI;
  }
  else if (endOri - startOri < M_PI)
  {
    endOri += 2 * M_PI;
  }
  // printf("endOri %f\n", endOri); //检验此时的endOri是否为正确范围

  bool halfPassed = false;
  int count = cloudSize; // 在后续分配scanID时用于计数
  PointType point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS); // Vector<类型> 标识符(最大容量）

  // 在新的雷达中，scanID、行号、列号可能已经可以直接获取，无需通过俯仰角计算
  // 计算每一个点的水平角和俯仰角
  for (int i = 0; i < cloudSize; i++)
  {
    point.x = laserCloudIn.points[i].x;
    point.y = laserCloudIn.points[i].y;
    point.z = laserCloudIn.points[i].z;

    // 计算俯仰角
    float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
    int scanID = 0;

    if (N_SCANS == 16)
    {                                       // 16线雷达范围为上下各15度,相邻scan的夹角为2度
      scanID = int((angle + 15) / 2 + 0.5); // 利用角度计算scanID，+0.5可以四舍五入
      if (scanID > (N_SCANS - 1) || scanID < 0)
      { // 不符合实际，删除该点
        count--;
        continue;
      }
    }
    else if (N_SCANS == 32)
    { // 原理同上，可自己查阅雷达手册修改计算scanID时的数值
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else if (N_SCANS == 64)
    {
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else
    {
      printf("wrong scan number\n");
      ROS_BREAK();
    }
    // printf("angle %f scanID %d \n", angle, scanID);

    // 计算水平角
    float ori = -atan2(point.y, point.x);
    if (!halfPassed)
    { // 原理同计算起始点和结束点的角度
      if (ori < startOri - M_PI / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > startOri + M_PI * 3 / 2)
      { // 这种情况在kitti中不会发生
        ori -= 2 * M_PI;
      }
      if (ori - startOri > M_PI)
      {
        halfPassed = true;
      }
    }
    else
    {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > endOri + M_PI / 2)
      {
        ori -= 2 * M_PI;
      }
    }

    float relTime = (ori - startOri) / (endOri - startOri); // 匀速假设下，该点的时间在该帧的比例
    point.intensity = scanID + scanPeriod * relTime;        // 整数部分是scan的索引，小数部分是相对起始时刻的时间
    // 根据scan的索引放入各自数组;最终的存储结果可以视为：点云存入一个二维矩阵，每个元素为一个点
    laserCloudScans[scanID].push_back(point);

    // 新的雷达会为点加上时间戳，不需要再计算
  }

  cloudSize = count; // 有效的点云的数目
  ////printf("points size %d \n", cloudSize);

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>()); // 用于存储所有点的一维数组

  for (int i = 0; i < N_SCANS; i++)
  { // 根据原文，用一个点左右五个点来计算粗糙度/平滑度/曲率，靠近边缘的点不能计算该值
    // 计算粗糙度的点的数量可以自己确定。共有1800左右个点，舍弃这个数量可以接受
    // 对于存放点云的二维数组，左右两侧各5列点不能用于计算曲率
    // 现在要将点全部存为一维，也要根据上述规则确定可以计算曲率的点
    // 可使用并行计算加速,可以参考lio-sam的并行计算方式
    scanStartInd[i] = laserCloud->size() + 5; // 位置为原二维矩阵中（第i+1行）的第六个点
    *laserCloud += laserCloudScans[i];        // 将每一行点云存入，laserCloud的大小增加量为一行点云的量
    scanEndInd[i] = laserCloud->size() - 6;   // 位置为原二维矩阵中（第i+1行）的倒数第六个点
  }

  //*****特征提取及均匀化*****
  ////printf("prepare time %f \n", t_prepare.toc());

  // 此处可以使用openMP并行计算以加速(可以参考lio-sam)
  for (int i = 5; i < cloudSize - 5; i++)
  { // 除了起始5点和结束5点都计算了曲率，实际上多计算了中间的边缘点
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
    // 存储曲率，索引
    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ; // 该点的曲率
    cloudSortInd[i] = i;                                               // 后续会有排序操作，在此记录该点原先的位置
    cloudNeighborPicked[i] = 0;                                        // 标志位，用于说明该点是否应避免选为特征点。
    cloudLabel[i] = 0;                                                 // 标志位，标识该点属于曲率最大、稍大、稍小或最小
  }

  TicToc t_pts; // 提取当前帧特征点所用的时间

  // 点的四种分类
  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  float t_q_sort = 0; // 用于记录排序所用的时间
  for (int i = 0; i < N_SCANS; i++)
  {                                          // 按行处理点云
    if (scanEndInd[i] - scanStartInd[i] < 6) // 因为后续要六等分，所以该行的点数不足（无有效点），到下一行
      continue;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>); // 用来存储不太平整的点
    for (int j = 0; j < 6; j++)
    {                                                                       // 等分为6个等份以保证均匀地提取特征
      int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; // 每个等份的起始和结束
      int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

      TicToc t_tmp; // 在此处创建时保存了时刻，即排序开始的时刻
      // sort函数：第一个是要排序的数组的起始地址。第二个是结束的地址（最后一位要排序的地址）
      // 第三个参数是排序的方法，可以是从大到小也可是从小到大，还可以不写第三个参数，此时默认的排序方法是从小到大排序。
      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp); // 按照曲率从小到大排序
      t_q_sort += t_tmp.toc();                                   // 加上本次排序的时间得到排序的总时间

      // 挑选边缘点
      int largestPickedNum = 0;
      for (int k = ep; k >= ep; k--) // 倒序遍历是为了从曲率最大处挑选
      {
        int ind = cloudSortInd[k]; // 该数组为该点的原始位置，即该点的索引
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
        { // 判断是否为有效点/边缘点
          largestPickedNum++;
          if (largestPickedNum <= 2)
          {                                                       // 每段选两个曲率大的点
            cloudLabel[ind] = 2;                                  // label为2是曲率最大的标记
            cornerPointsSharp.push_back(laserCloud->points[ind]); // 曲率最大的点
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          }
          else if (largestPickedNum <= 20)
          {                      // 20个曲率稍微大一些的点
            cloudLabel[ind] = 1; // label为1是曲率稍微大一些的标记
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          }
          else
          {
            break; // 已经选取了足够的边缘点，跳出该for循环
          }

          cloudNeighborPicked[ind] = 1; // 此时该点已经选中；置为1,表示避免选取
          // 为保证特征点不集中，选中的特征点周围的点避免选取。
          // 该周围指的是物理空间意义上的周围，不是存储空间意义上的周围。
          for (int l = 1; l <= 5; l++)
          { // 从该点向右判断5个点
            // 查看相邻点的距离差距是否过大，如果距离过大说明点云不连续，存储空间中更远的点已经不属于该点物理空间的周围
            // 有两种不连续情况：剔除了nan点后，相邻的两点可能距离过大；如果是（面/线）特征边缘，也会导致距离过大
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
              break;

            cloudNeighborPicked[ind + l] = 1; // 物理空间意义的周围点避免选取
          }
          for (int l = -1; l >= -5; l--)
          { /// 从该点向左判断5个点
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      // 开始挑选平面点
      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++)
      {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
        {
          cloudLabel[ind] = -1;
          surfPointsFlat.push_back(laserCloud->points[ind]);
          smallestPickedNum++;
          if (smallestPickedNum >= 4)
            break;

          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
              break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++)
      {
        if (cloudLabel[k] <= 0)
        { // 因为实际中面点远多余角点，因此全部视为面点更为合理；当然仍然有部分角点被误归为面点
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }
    // 此时已经完成该行特征的选取

    // 对未选中的一般点用体素滤波器降采样
    pcl::PointCloud<PointType> surfPointsLessFlatScanDS; // 降采样后点
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2); // 正方体变长为0.2，其中的点取重心并保存
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    surfPointsLessFlat += surfPointsLessFlatScanDS;
    // 继续处理下一行
  }
  ////printf("sort q time %f \n", t_q_sort); //输出排序总时间
  ////printf("seperate points time %f \n", t_pts.toc());

  // 分别发布当前点云、四种特征点云
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/camera_init";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsSharpMsg.header.frame_id = "/camera_init";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
  cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsFlat2.header.frame_id = "/camera_init";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/camera_init";
  /// printf("scan2 HW:%d %d\n",surfPointsLessFlat2.height, surfPointsLessFlat2.width);
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

  // pub each scan
  // 可以按照每个scan发送，但此处为false
  if (PUB_EACH_LINE)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      sensor_msgs::PointCloud2 scanMsg;
      pcl::toROSMsg(laserCloudScans[i], scanMsg);
      scanMsg.header.stamp = laserCloudMsg->header.stamp;
      scanMsg.header.frame_id = "/camera_init";
      pubEachScan[i].publish(scanMsg);
    }
  }

  ////printf("scan registration time %f ms *************\n", t_whole.toc()); //该帧的总处理时间
  if (t_whole.toc() > 100)
  {
    ROS_WARN("scan registration process over 100ms"); // 处理时间过长可能会导致丢帧
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration"); // 创建ros节点
  ros::NodeHandle nh;                        // 创建句柄

  // 从参数服务器获取参数；参数可能写在launch文件中
  // 从配置文件中获取激光雷达的线数，默认为16
  nh.param<int>("scan_line", N_SCANS, 16);
  // 与雷达距离小于该距离的点不考虑；因为这些点可能是安装该雷达的载体，例如汽车;默认0.1
  nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

  // 输出线的数量（运行前显示关键参数），可用于确认线数是否正确
  printf("scan line number %d \n", N_SCANS);
  if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
  {
    printf("only support velodyne with 16, 32 or 64 scan line!");
    return 0;
  }
  // laserCloud Handler为回调函数；每次接收到消息时，执行回调函数
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);
  pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

  if (PUB_EACH_LINE)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
      pubEachScan.push_back(tmp);
    }
  }
  ros::spin();

  return 0;
}