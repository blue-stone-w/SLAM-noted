/*
Description: done
*/

#ifndef INCLUDE_MAPRINGBUFFER_H_
#define INCLUDE_MAPRINGBUFFER_H_

#include <iostream>
#include <map>

template <typename Meas> // Meas is data type, not a variable; to construct a class template
class MapRingBuffer
{
 public:
  // <key, value>, key is a unique index to find associated value, here it's time
  std::map<double, Meas> measMap_;
  // iterator is a data type that examines and traverses the elements
  typename std::map<double, Meas>::iterator itMeas_;

  int size; // 保存的点云的ID的数量
  double maxWaitTime_; //---
  double minWaitTime_; //---

  MapRingBuffer( )
  {
    maxWaitTime_ = 0.1;
    minWaitTime_ = 0.0;
  }

  virtual ~MapRingBuffer( ) {}

  bool allocate(const int sizeBuffer)
  {
    if (sizeBuffer <= 0)
    {
      return false;
    }
    else
    {
      size = sizeBuffer;
      return true;
    }
  }

  int getSize( )
  {
    return measMap_.size( );
  }

  // update: add new and erase old to limit size of map
  // 基类的引用，可以接受继承类作为输入
  void addMeas(const Meas &meas, const double &t)
  {
    measMap_.insert(std::make_pair(t, meas));

    // ensure the size of the map, and remove the oldest element
    if (measMap_.size( ) > size)
    {
      measMap_.erase(measMap_.begin( ));
    }
  }

  void clear( ) { measMap_.clear( ); }

  void clean(double t)
  { // erase old data
    while (measMap_.size( ) >= 1 && measMap_.begin( )->first <= t)
    {
      measMap_.erase(measMap_.begin( ));
    }
  }


  bool getNextTime(double actualTime, double &nextTime)
  { // 获取第一个大于actualTime的时间
    // upper_bound: return a iterator pointing to first element that "key > actualTime"
    itMeas_ = measMap_.upper_bound(actualTime);
    if (itMeas_ != measMap_.end( ))
    {
      nextTime = itMeas_->first;
      return true;
    }
    else
    {
      return false;
    }
  }
  // 没有使用---
  void waitTime(double actualTime, double &time)
  {
    double measurementTime = actualTime - maxWaitTime_;
    // rbegin: return a reversed iterator pointing to last element and increasing when moving reversely
    if (!measMap_.empty( ) && measMap_.rbegin( )->first + minWaitTime_ > measurementTime)
    {
      measurementTime = measMap_.rbegin( )->first + minWaitTime_;
    }
    if (time > measurementTime)
    {
      time = measurementTime; //---
    }
  }


  bool getFirstTime(double &firstTime)
  {
    if (!measMap_.empty( ))
    {
      firstTime = measMap_.begin( )->first;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool getFirstMeas(Meas &firstMeas)
  {
    if (!measMap_.empty( ))
    {
      firstMeas = measMap_.begin( )->second;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool getLastTime(double &lastTime)
  {
    if (!measMap_.empty( ))
    {
      lastTime = measMap_.rbegin( )->first;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool getLastMeas(Meas &lastMeas)
  {
    if (!measMap_.empty( ))
    {
      lastMeas = measMap_.rbegin( )->second;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool getLastLastMeas(Meas &lastlastMeas)
  {
    if (measMap_.size( ) >= 2)
    {
      auto itr = measMap_.rbegin( );
      itr++;
      lastlastMeas = itr->second;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool hasMeasurementAt(double t)
  {
    // count: return num of element with key=t, key is unique, so return 0 or 1
    return measMap_.count(t) > 0;
  }

  bool empty( ) { return measMap_.empty( ); }

  void printContainer( )
  {
    itMeas_ = measMap_.begin( );
    while (measMap_.size( ) >= 1 && itMeas_ != measMap_.end( ))
    {
      std::cout << itMeas_->second << " ";
      itMeas_++;
    }
  }
};

#endif // INCLUDE_MAPRINGBUFFER_H_