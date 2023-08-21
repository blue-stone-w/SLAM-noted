/*
Description: done
*/
#include <parameters.h>

namespace parameter
{

// enable calibration
int CALIBRATE_IMU; // indicate calibration status, used when read parameter from file .yaml
int SHOW_CONFIGURATION;
int AVERAGE_NUMS;

// initial parameters
double IMU_LIDAR_EXTRINSIC_ANGLE;
double IMU_MISALIGN_ANGLE;

// lidar parameters
int LINE_NUM; // when we get range image, line is row num and scan is column(列) num.
int SCAN_NUM; // column num.
double SCAN_PERIOD;
double EDGE_THRESHOLD;
double SURF_THRESHOLD;
double NEAREST_FEATURE_SEARCH_SQ_DIST;

// test
int VERBOSE; // whether publish info during running
int ICP_FREQ;
int MAX_LIDAR_NUMS;
int NUM_ITER;
double LIDAR_SCALE;
double LIDAR_STD;

// sub topic name
std::string IMU_TOPIC;
std::string LIDAR_TOPIC;

// pub topic name
std::string LIDAR_ODOMETRY_TOPIC;
std::string LIDAR_MAPPING_TOPIC;

// kalman filter
double ACC_N; // accelerate noise
double ACC_W;
double GYR_N; // gyroscope noise(angluar velocity noise)
double GYR_W;
V3D INIT_POS_STD; // V3D and Q4D is from parameters.h
V3D INIT_VEL_STD;
V3D INIT_ATT_STD; // attitude(angle)
V3D INIT_ACC_STD;
V3D INIT_GYR_STD;

// initial IMU biases
V3D INIT_BA;
V3D INIT_BW;

// extrinsic parameters
V3D INIT_TBL;
Q4D INIT_RBL;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  T ans;
  if (n.getParam(name, ans))
  {
    // ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown( );
  }
  return ans;
}

// 读取参数到本文件中的全局变量中。
// 为了在其他文件中使用读进来的变量，需要extern这些变量，恰好将extern写在了本文件的头文件中以便一同使用
void readParameters(ros::NodeHandle &n)
{
  std::string config_file;
  config_file = readParam<std::string>(n, "config_file"); // 从参数服务器获取配置参数的路径
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ); // 从该路径加载所有的参数
  if (!fsSettings.isOpened( ))
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  CALIBRATE_IMU                  = fsSettings["calibrate_imu"];
  SHOW_CONFIGURATION             = fsSettings["show_configuration"];
  AVERAGE_NUMS                   = fsSettings["average_nums"];
  IMU_LIDAR_EXTRINSIC_ANGLE      = fsSettings["imu_lidar_extrinsic_angle"];
  IMU_MISALIGN_ANGLE             = fsSettings["imu_misalign_angle"];
  LINE_NUM                       = fsSettings["line_num"];
  SCAN_NUM                       = fsSettings["scan_num"];
  SCAN_PERIOD                    = fsSettings["scan_period"];
  EDGE_THRESHOLD                 = fsSettings["edge_threshold"];
  SURF_THRESHOLD                 = fsSettings["surf_threshold"];
  NEAREST_FEATURE_SEARCH_SQ_DIST = fsSettings["nearest_feature_search_sq_dist"];
  VERBOSE                        = fsSettings["verbose"];
  ICP_FREQ                       = fsSettings["icp_freq"];
  MAX_LIDAR_NUMS                 = fsSettings["max_lidar_nums"];
  NUM_ITER                       = fsSettings["num_iter"];
  LIDAR_SCALE                    = fsSettings["lidar_scale"];
  LIDAR_STD                      = fsSettings["lidar_std"];

  fsSettings["imu_topic"] >> IMU_TOPIC; // data type string
  fsSettings["lidar_topic"] >> LIDAR_TOPIC;
  fsSettings["lidar_odometry_topic"] >> LIDAR_ODOMETRY_TOPIC;
  fsSettings["lidar_mapping_topic"] >> LIDAR_MAPPING_TOPIC;

  ACC_N = fsSettings["acc_n"]; // data type double
  ACC_W = fsSettings["acc_w"];
  GYR_N = fsSettings["gyr_n"];
  GYR_W = fsSettings["gyr_w"];

  readV3D(&fsSettings, "init_pos_std", INIT_POS_STD); // data type is matrix
  readV3D(&fsSettings, "init_vel_std", INIT_VEL_STD);
  readV3D(&fsSettings, "init_att_std", INIT_ATT_STD);
  readV3D(&fsSettings, "init_acc_std", INIT_ACC_STD);
  readV3D(&fsSettings, "init_gyr_std", INIT_GYR_STD);

  readV3D(&fsSettings, "init_ba", INIT_BA);
  readV3D(&fsSettings, "init_bw", INIT_BW);
  readV3D(&fsSettings, "init_tbl", INIT_TBL);
  readQ4D(&fsSettings, "init_rbl", INIT_RBL);
}

void readV3D(cv::FileStorage *file, const std::__cxx11::string &name, V3D &vec_eigen)
{
  cv::Mat vec_cv;
  (*file)[name] >> vec_cv;
  cv::cv2eigen(vec_cv, vec_eigen); // transform cv matrix to vector
}

void readQ4D(cv::FileStorage *file, const std::__cxx11::string &name, Q4D &quat_eigen)
{
  cv::Mat mat_cv;
  (*file)[name] >> mat_cv;
  M3D mat_eigen;
  cv::cv2eigen(mat_cv, mat_eigen); // transform cv matrix to normal/eigen matrix
  Q4D quat(mat_eigen); // transform matrix to quaternion
  quat_eigen = quat.normalized( );
}
} // namespace parameter