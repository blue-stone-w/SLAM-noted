/*
Description: done
*/

#ifndef INCLUDE_UTILITY_H_
#define INCLUDE_UTILITY_H_

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cloud_msgs/cloud_info.h"

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI PointType;

// VLP-16
extern const int N_SCAN        = 16;
extern const int Horizon_SCAN  = 1800;
extern const float ang_res_x   = 0.2; // horizonal angle resolution
extern const float ang_res_y   = 2.0; // vertical
extern const float ang_bottom  = 15.0 + 0.1;
extern const int groundScanInd = 5;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag    = true;
extern const double mappingProcessInterval = 0.3; // use in mapping node to limit frequence

extern const float scanPeriod = 0.1; // lidar cloud period
extern const int systemDelay  = 0; // time for system to restart
extern const int imuQueLength = 200;
extern const string imuTopic  = "/imu/data";

extern const float sensorMountAngle   = 0.0; // 雷达水平面与地面的夹角
extern const float segmentTheta       = 1.0472;
extern const int segmentValidPointNum = 5; // segment contain points at least this num
extern const int segmentValidLineNum  = 3; // segment span lines at least this num
extern const float segmentAlphaX      = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY      = ang_res_y / 180.0 * M_PI;

extern const int edgeFeatureNum               = 2;
extern const int surfFeatureNum               = 4;
extern const int sectionsTotal                = 6; // range image is divided into 6 segments
extern const float edgeThreshold              = 0.5;
extern const float surfThreshold              = 0.5;
extern const float nearestFeatureSearchSqDist = 25; // 未使用

extern const float surroundingKeyframeSearchRadius = 50.0; // look for key frames in this radius
extern const int surroundingKeyframeSearchNum      = 50; // extract key frame at least this num

extern const float historyKeyframeSearchRadius = 5.0; // look for loop frame in this radius
extern const int historyKeyframeSearchNum      = 25; // look for history key frame of loop(prev frame) in this num
extern const float historyKeyframeFitnessScore = 0.3; // threshold to judge whether loop ICP result is acceptable

extern const float globalMapVisualizationSearchRadius = 500.0; // look for frame in this radius around robot to visualize

struct smoothness_t
{
  float value;
  size_t ind; // index of cooresponding point
};

// lambda for compare smoothness
struct by_value
{
  bool operator( )(smoothness_t const &left, smoothness_t const &right)
  {
    return left.value < right.value;
  }
};

#endif // INCLUDE_UTILITY_H_