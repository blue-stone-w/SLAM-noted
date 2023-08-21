#ifndef FRAME_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define FRAME_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace frame {
class IMUData {
  public:
    //线速度
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };
    //角速度
    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    //方向
    class Orientation {
      public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

      public:
        //归一化
        void Normlize() {
          double norm  = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0)) +pow(w, 2.0);
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    //声明变量
    double time =0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;

  public:
    //把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};
}
#endif