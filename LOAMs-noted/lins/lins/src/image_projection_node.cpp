/* done
Description: range image, label, segment
*/

#include <parameters.h>

using namespace parameter;

class ImageProjection
{
 private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  ros::Subscriber subLaserCloud;

  ros::Publisher pubFullCloud;
  ros::Publisher pubFullInfoCloud;

  ros::Publisher pubGroundCloud;
  ros::Publisher pubSegmentedCloud; // selected points
  ros::Publisher pubSegmentedCloudPure; // selected points and label
  ros::Publisher pubSegmentedCloudInfo;
  ros::Publisher pubOutlierCloud;

  pcl::PointCloud<PointType>::Ptr laserCloudIn;

  pcl::PointCloud<PointType>::Ptr fullCloud;
  pcl::PointCloud<PointType>::Ptr fullInfoCloud; // cloud with info, info = intensity = range

  pcl::PointCloud<PointType>::Ptr groundCloud;
  pcl::PointCloud<PointType>::Ptr segmentedCloud; // 被分类后的点(包括地面)
  pcl::PointCloud<PointType>::Ptr segmentedCloudPure; // 保存有效标签对应的点
  pcl::PointCloud<PointType>::Ptr outlierCloud; // 未分类的有效点(聚类失败的点中非地面点)

  PointType nanPoint; // 用于初始化点云;PointType is from parameters.h

  cv::Mat rangeMat;
  cv::Mat labelMat; // 0:未分类
  cv::Mat groundMat;
  int labelCount; // 计数当前点云的label的数量，同时也作为对应seg点云的标签名

  float startOrientation;
  float endOrientation;

  cloud_msgs::cloud_info segMsg; // save all selected points from this frame(range image) and their property/info
  std_msgs::Header cloudHeader; // literally

  std::vector<std::pair<uint8_t, uint8_t>> neighborIterator;

  uint16_t *allPushedIndX; // label的过程中，用于保存点的行号
  uint16_t *allPushedIndY; // label的过程中，用于保存点的列号

  uint16_t *queueIndX; // label的广度优先搜索时，缓存的点云队列中的点的行号
  uint16_t *queueIndY; // 列号

 public:
  ImageProjection(ros::NodeHandle &nh, ros::NodeHandle &pnh) :
    nh(nh), pnh(pnh)
  { // send arguments to constructor
    // (topic name, num of saved msg, callback function, 传入的参数)
    subLaserCloud = pnh.subscribe<sensor_msgs::PointCloud2>(LIDAR_TOPIC, 1, &ImageProjection::cloudHandler, this);

    pubFullCloud     = pnh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
    pubFullInfoCloud = pnh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

    pubGroundCloud        = pnh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
    pubSegmentedCloud     = pnh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
    pubSegmentedCloudPure = pnh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
    pubSegmentedCloudInfo = pnh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);
    pubOutlierCloud       = pnh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);

    nanPoint.x         = std::numeric_limits<float>::quiet_NaN( ); // define nanpoint
    nanPoint.y         = std::numeric_limits<float>::quiet_NaN( );
    nanPoint.z         = std::numeric_limits<float>::quiet_NaN( );
    nanPoint.intensity = -1;

    allocateMemory( );
    resetParameters( );
  }

  void allocateMemory( )
  {
    laserCloudIn.reset(new pcl::PointCloud<PointType>( ));

    fullCloud.reset(new pcl::PointCloud<PointType>( ));
    fullInfoCloud.reset(new pcl::PointCloud<PointType>( ));

    groundCloud.reset(new pcl::PointCloud<PointType>( ));
    segmentedCloud.reset(new pcl::PointCloud<PointType>( ));
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>( ));
    outlierCloud.reset(new pcl::PointCloud<PointType>( ));

    fullCloud->points.resize(LINE_NUM * SCAN_NUM);
    fullInfoCloud->points.resize(LINE_NUM * SCAN_NUM);

    segMsg.startRingIndex.assign(LINE_NUM, 0); // (size, initial value)
    segMsg.endRingIndex.assign(LINE_NUM, 0);

    segMsg.segmentedCloudGroundFlag.assign(LINE_NUM * SCAN_NUM, false);
    segMsg.segmentedCloudColInd.assign(LINE_NUM * SCAN_NUM, 0);
    segMsg.segmentedCloudRange.assign(LINE_NUM * SCAN_NUM, 0);

    // one point can have four neighbors in range image from four directions
    std::pair<int8_t, int8_t> neighbor; // this variable indicates neighbor's direction
    neighbor.first  = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor); // store four directions
    neighbor.first  = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first  = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first  = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    // when searching all points with same label, save these points temporarily
    allPushedIndX = new uint16_t[LINE_NUM * SCAN_NUM];
    allPushedIndY = new uint16_t[LINE_NUM * SCAN_NUM];

    // coordinates of points with the same label are saved in this queue
    queueIndX = new uint16_t[LINE_NUM * SCAN_NUM];
    queueIndY = new uint16_t[LINE_NUM * SCAN_NUM];
  }

  void resetParameters( )
  {
    laserCloudIn->clear( );
    groundCloud->clear( );
    segmentedCloud->clear( );
    segmentedCloudPure->clear( );
    outlierCloud->clear( );

    /**********************************************************
    CV_<bit_depth>(S|U|F)C<number_of_channels>
    channels = 1：灰度图片--grayImg---是--单通道图像
    channels = 3：RGB彩色图像---------是--3通道图像
    channels = 4：带Alph通道的RGB图像--是--4通道图像
    S--代表---signed int---有符号整形
    U--代表--unsigned int--无符号整形
    F--代表--float---------单精度浮点型
    **********************************************************/

    // SCAN_NUM行LINE_NUM列的图像矩阵, CV_32F表示每个元素的值的类型为32位浮点数(default 单channel), 矩阵每个元素都赋值为FLT_MAX
    rangeMat = cv::Mat(LINE_NUM, SCAN_NUM, CV_32F, cv::Scalar::all(FLT_MAX));
    // cv::Scalar的构造函数是cv::Scalar(v1, v2, v3, v4)，参数依次是BGR&图片的透明度，
    groundMat  = cv::Mat(LINE_NUM, SCAN_NUM, CV_8S, cv::Scalar::all(0));
    labelMat   = cv::Mat(LINE_NUM, SCAN_NUM, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    // std::fill函数的作用是：将一个区间的元素都赋予指定的值，即在[first, last)范围内填充指定值。
    std::fill(fullCloud->points.begin( ), fullCloud->points.end( ), nanPoint);
    std::fill(fullInfoCloud->points.begin( ), fullInfoCloud->points.end( ), nanPoint);
  }

  ~ImageProjection( ) {}

  void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    cloudHeader = laserCloudMsg->header;
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    std::vector<int> indices;
    // 三个参数，分别为输入点云，输出点云,保留的点的索引
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
  {
    TicToc ts_total; // measure process time
    copyPointCloud(laserCloudMsg); // 转成pcl点云，去除NaN
    findStartEndAngle( ); // 计算起止角度和角度范围
    projectPointCloud( ); // project point cloud to range image; 投影到有序的image中，并记录点的行列和深度
    groundRemoval( ); // 确定属于地面的pixel，不用参与聚类
    cloudSegmentation( ); // 非地面点的聚类，聚类点和地面点以线为单位重新组织成紧凑的形式
    publishCloud( );
    resetParameters( );
    double time_total = ts_total.toc( );
  }

  void findStartEndAngle( )
  {
    segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size( ) - 1].y,
                                     laserCloudIn->points[laserCloudIn->points.size( ) - 2].x)
                            + 2 * M_PI;
    // to ensure that end Orientation is in logical interval
    if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI)
    {
      segMsg.endOrientation -= 2 * M_PI;
    }
    else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
    {
      segMsg.endOrientation += 2 * M_PI;
    }

    segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
  }

  void projectPointCloud( )
  { // project point cloud to range image
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    PointType thisPoint;

    cloudSize = laserCloudIn->points.size( );

    for (size_t i = 0; i < cloudSize; ++i)
    {
      thisPoint.x = laserCloudIn->points[i].x;
      thisPoint.y = laserCloudIn->points[i].y;
      thisPoint.z = laserCloudIn->points[i].z;

      verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
      rowIdn        = (verticalAngle + ang_bottom) / ang_res_y; // ang_bottom is depended on lidar, here 为垂直角度范围的一半;
      if (rowIdn < 0 || rowIdn >= LINE_NUM)
        continue;

      horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
      columnIdn    = -round((horizonAngle - 90.0) / ang_res_x) + SCAN_NUM / 2; // 从x轴负方向逆时针分别为0~SCAN_NUM-1
      if (columnIdn >= SCAN_NUM)
        columnIdn -= SCAN_NUM;
      if (columnIdn < 0 || columnIdn >= SCAN_NUM)
        continue;

      range                                 = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
      rangeMat.at<float>(rowIdn, columnIdn) = range;

      thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0; // default -1 ; time in this frame

      index                    = columnIdn + rowIdn * SCAN_NUM; // order by row; lower point has lower index
      fullCloud->points[index] = thisPoint;

      fullInfoCloud->points[index].intensity = range;
    }
  }

  void groundRemoval( )
  {
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;

    for (size_t j = 0; j < SCAN_NUM; ++j)
    {
      for (size_t i = 0; i < groundScanInd; ++i)
      {
        lowerInd = j + i * SCAN_NUM;
        upperInd = j + (i + 1) * SCAN_NUM;

        // 遇到没有点的pixel; at least one of two points are NaNpoint
        if (fullCloud->points[lowerInd].intensity == -1 || fullCloud->points[upperInd].intensity == -1)
        {
          groundMat.at<int8_t>(i, j) = -1;
          continue;
        }

        diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
        diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
        diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

        angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

        // 角度足够朝下的点为地面点
        if (abs(angle - sensorMountAngle) <= 10) // 减掉雷达水平面与地面的夹脚
        {
          groundMat.at<int8_t>(i, j)     = 1;
          groundMat.at<int8_t>(i + 1, j) = 1;
        }
      }
    }

    for (size_t i = 0; i < LINE_NUM; ++i)
    {
      for (size_t j = 0; j < SCAN_NUM; ++j)
      {
        if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX)
        { // point is ground or too far
          labelMat.at<int>(i, j) = -1; // is indicated as invalid point, dont extract features from these points
          // 无效pixel和地面不会再参与聚类
        }
      }
    }

    if (pubGroundCloud.getNumSubscribers( ) != 0)
    {
      for (size_t i = 0; i <= groundScanInd; ++i)
      {
        for (size_t j = 0; j < SCAN_NUM; ++j)
        {
          // 获取地面点云
          if (groundMat.at<int8_t>(i, j) == 1)
            groundCloud->push_back(fullCloud->points[j + i * SCAN_NUM]);
        }
      }
    }
  }

  void cloudSegmentation( )
  {
    for (size_t i = 0; i < LINE_NUM; ++i)
      for (size_t j = 0; j < SCAN_NUM; ++j)
        if (labelMat.at<int>(i, j) == 0)
          labelComponents(i, j); // 广度优先搜索聚类，将label信息标注在labelMat中
    // now every point is labeled

    int sizeOfSegCloud = 0; // num of selected points
    for (size_t i = 0; i < LINE_NUM; ++i)
    {
      // 记录每线在fullCloud中的每一行的起始索引; we dont extract features from first 5 points
      segMsg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

      for (size_t j = 0; j < SCAN_NUM; ++j)
      {
        // this point isn't nanpoint; 地面或者聚类点
        if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1)
        {
          // points with invalid label
          if (labelMat.at<int>(i, j) == 999999)
          {
            // this point is not ground and 等角度降采样
            if (i > groundScanInd && j % 5 == 0)
            { // this point will be discarded
              outlierCloud->push_back(fullCloud->points[j + i * SCAN_NUM]);
              continue;
            }
            else
            {
              continue;
            }
          }
          // 对于地面点进行等角度降采样，连接处不计
          if (groundMat.at<int8_t>(i, j) == 1)
          {
            if (j % 5 != 0 && j > 5 && j < SCAN_NUM - 5)
              continue;
          }

          // save property of every selected points
          // 是否为地面点
          segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i, j) == 1); // 是否为地面
          // 指示当前点的在距离图像上的列数
          segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
          segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i, j); // range
          // 分割点,包含所有有效的聚类; the selected points, label isn't saved
          segmentedCloud->push_back(fullCloud->points[j + i * SCAN_NUM]);
          ++sizeOfSegCloud;
        }
      }
      // we dont extract features from last 5 points
      segMsg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
    } //  now we have selected and saved all needed points
    // 将所有非地面点且为有效聚类的点 组成  segmentedCloudPure
    if (pubSegmentedCloudPure.getNumSubscribers( ) != 0)
    {
      for (size_t i = 0; i < LINE_NUM; ++i)
      {
        for (size_t j = 0; j < SCAN_NUM; ++j)
        {
          if (labelMat.at<int>(i, j) > 0 && labelMat.at<int>(i, j) != 999999)
          {
            segmentedCloudPure->push_back(fullCloud->points[j + i * SCAN_NUM]);
            segmentedCloudPure->points.back( ).intensity = labelMat.at<int>(i, j); // label is saved
          }
        }
      }
    }
  }

  void labelComponents(int row, int col)
  { // row and col is position in range image
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY; // coordinate of a point and his neighbor point(range image)
    bool lineCountFlag[LINE_NUM] = {false}; // whether this segment is distributed on this line

    queueIndX[0]      = row; // coordinates of points with the same label are saved in this queue
    queueIndY[0]      = col;
    int queueSize     = 1; // num of points, with the same label but we haven searched his neighbor; 没有检查的剩余长度
    int queueStartInd = 0; // 没有检查的起始索引
    int queueEndInd   = 1; // 新的空位置

    allPushedIndX[0]     = row;
    allPushedIndY[0]     = col;
    int allPushedIndSize = 1; // 总共的点数

    // find all points with the same label; 广度有限搜索
    while (queueSize > 0)
    { // there are points that we should search his neighbor
      fromIndX = queueIndX[queueStartInd];
      fromIndY = queueIndY[queueStartInd];
      --queueSize;
      ++queueStartInd;
      labelMat.at<int>(fromIndX, fromIndY) = labelCount; // label

      // looking for fromIndX's neighbors and save them
      for (auto iter = neighborIterator.begin( ); iter != neighborIterator.end( ); ++iter)
      {
        thisIndX = fromIndX + (*iter).first;
        thisIndY = fromIndY + (*iter).second;

        if (thisIndX < 0 || thisIndX >= LINE_NUM)
          continue; // this point lies on side of range image

        // range image has left and right sides, but original cloud is not, 列是循环的
        if (thisIndY < 0)
          thisIndY = SCAN_NUM - 1;
        if (thisIndY >= SCAN_NUM)
          thisIndY = 0;

        if (labelMat.at<int>(thisIndX, thisIndY) != 0)
          continue; // this point has been labeled, 地面点和无效点不参与聚类

        d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));
        d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                      rangeMat.at<float>(thisIndX, thisIndY));

        if ((*iter).first == 0) // neighbor direction; 同一行
          alpha = segmentAlphaX;
        else // 同一列
          alpha = segmentAlphaY;

        angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha))); // angle of two adjacent points

        if (angle > segmentTheta)
        { // two points are adjacent; 角度比较大，说明是比较连续的面，入射角可以很大
          queueIndX[queueEndInd] = thisIndX;
          queueIndY[queueEndInd] = thisIndY;
          ++queueSize;
          ++queueEndInd;

          labelMat.at<int>(thisIndX, thisIndY) = labelCount; // 标注上label
          lineCountFlag[thisIndX]              = true; // 占用了一条线

          allPushedIndX[allPushedIndSize] = thisIndX;
          allPushedIndY[allPushedIndSize] = thisIndY;
          ++allPushedIndSize;
        }
      }
    } // now we have found all points with the same label with input point

    // judge validity of the segment: 总点数比较多，或者占的线数比较多
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
      feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum)
    {
      int lineCount = 0;
      for (size_t i = 0; i < LINE_NUM; ++i)
        if (lineCountFlag[i] == true)
          ++lineCount;
      // because resolution on X and Y direction are different,
      // if points distributed in more than segmentValidLineNum lines, this segment is valid
      if (lineCount >= segmentValidLineNum)
        feasibleSegment = true;
    }

    // 为下次做准备
    if (feasibleSegment == true)
    {
      ++labelCount;
    }
    else
    {
      for (size_t i = 0; i < allPushedIndSize; ++i)
      {
        // discard all points in this segment; 不符合要求的label被修改成9999999
        labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
      }
    }
  }

  void publishCloud( )
  {
    segMsg.header = cloudHeader;
    pubSegmentedCloudInfo.publish(segMsg);

    sensor_msgs::PointCloud2 laserCloudTemp;

    pcl::toROSMsg(*outlierCloud, laserCloudTemp);
    laserCloudTemp.header.stamp    = cloudHeader.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pubOutlierCloud.publish(laserCloudTemp);

    pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
    laserCloudTemp.header.stamp    = cloudHeader.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pubSegmentedCloud.publish(laserCloudTemp);

    if (pubFullCloud.getNumSubscribers( ) != 0)
    {
      pcl::toROSMsg(*fullCloud, laserCloudTemp);
      laserCloudTemp.header.stamp    = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubFullCloud.publish(laserCloudTemp);
    }

    if (pubGroundCloud.getNumSubscribers( ) != 0)
    {
      pcl::toROSMsg(*groundCloud, laserCloudTemp);
      laserCloudTemp.header.stamp    = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubGroundCloud.publish(laserCloudTemp);
    }

    if (pubSegmentedCloudPure.getNumSubscribers( ) != 0)
    {
      pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
      laserCloudTemp.header.stamp    = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubSegmentedCloudPure.publish(laserCloudTemp);
    }

    if (pubFullInfoCloud.getNumSubscribers( ) != 0)
    { // 只在intensity保存了range
      pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
      laserCloudTemp.header.stamp    = cloudHeader.stamp;
      laserCloudTemp.header.frame_id = "base_link";
      pubFullInfoCloud.publish(laserCloudTemp);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_projection_node");
  ros::NodeHandle nh; // nh is global namespace
  ros::NodeHandle pnh("~"); // pnh is local namespace

  parameter::readParameters(pnh); // params.cpp读取参数到全局变量，并且在params头文件中已经声明extern
  // 在params.h中声明并初始化的常值变量可以直接使用
  // 在params.cpp中声明的变量需要调用读取函数进行初始化，再借助头文件中的extern才能被使用

  ImageProjection featureHandler(nh, pnh); // get everything done in constructor

  ROS_INFO("\033[1;32m---->\033[0m Feature Extraction Module Started.");

  ros::spin( );
  return 0;
}