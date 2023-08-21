/* done
Description: class: IMU GPS Odometry EarthParams
*/

#ifndef INCLUDE_SENSOR_UTILS_HPP_
#define INCLUDE_SENSOR_UTILS_HPP_

#include <math_utils.h>
#include <parameters.h>

#include <iostream>
#include <map>

using namespace std;
using namespace math_utils;
using namespace parameter;

namespace sensor_utils
{

// Sensor measurement class
class Measurement
{
 public:
  Measurement( ) {}
  virtual ~Measurement( ) {}
};

class Imu : public Measurement
{
 public:
  Imu( ) {}
  Imu(double time, const V3D &acc, const V3D &gyr) :
    time(time), acc(acc), gyr(gyr) {}
  ~Imu( ) {}
  double time;
  V3D acc; // accelerometer measurement (m^2/sec)
  V3D gyr; // gyroscope measurement (rad/s)
};

class Gps : public Measurement
{
 public:
  Gps( )
  {
    time = 0.0;
    lat  = 0.0;
    lon  = 0.0;
    alt  = 0.0;
  }
  Gps(double time, double lat, double lon, double alt) :
    time(time), lat(lat), lon(lon), alt(alt) {}
  ~Gps( ) {}
  int status;
  double time;
  double lat;
  double lon;
  double alt;
  inline V3D pn( ) const { return V3D(lat, lon, alt); } // no varible will be changed in this function.
};

class Odometry : public Measurement
{
 public:
  Odometry( )
  {
    time = 0.0;
    rotation.setIdentity( );
    translation.setZero( );
  }
  Odometry(double time, const Q4D &rotation, const V3D &translation) :
    time(time), rotation(rotation), translation(translation) {}
  ~Odometry( ) {}

  Odometry inverse(const Odometry &odom)
  {
    Odometry inv;
    inv.time        = odom.time;
    inv.rotation    = odom.rotation.inverse( );
    inv.translation = -inv.rotation.toRotationMatrix( ) * odom.translation;
    return inv;
  }

  void boxPlus(const Odometry &increment)
  {
    // t2 = R*dt + t1
    translation = rotation.toRotationMatrix( ) * increment.translation + translation;
    rotation    = rotation * increment.rotation;
  }

  // box is define in article
  Odometry boxMinus(const Odometry &odom)
  {
    Odometry res;
    res.translation = odom.rotation.inverse( ).toRotationMatrix( ) * translation - odom.rotation.inverse( ).toRotationMatrix( ) * odom.translation;
    res.rotation    = odom.rotation.inverse( ) * rotation;
    return res;
  }
  double time;
  Q4D rotation;
  V3D translation;
};

// 没有使用
class EarthParams
{
 public:
  EarthParams( ) {}
  ~EarthParams( ) {}
  // 没有使用
  static M3D getDrp(const V3D &pn)
  {
    M3D Drp          = M3D::Zero( ); // 没有使用
    double latitude  = pn(0);
    double longitude = pn(1);
    double height    = pn(2);
    double sq        = 1 - EeEe * sin(latitude) * sin(latitude); // ratio: distance from earth center to surface
    double RNh       = Re / sqrt(sq) + height; // 地心到当前物体的高度的距离=地心到地面高度+物体到地面的高度
    double RMh       = RNh * (1 - EeEe) / sq + height; // (2.4)???
    Drp << 0, 1.0 / RMh, 0, 1.0 / (RNh * cos(latitude)), 0, 0, 0, 0, 1;
    return Drp;
  }

  // 没有使用
  static M3D getDpr(const V3D &pn)
  {
    M3D Drp = getDrp(pn);
    return Drp.inverse( );
  }
};

} // namespace sensor_utils

#endif // INCLUDE_SENSOR_UTILS_HPP_