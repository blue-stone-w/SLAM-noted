#https://blog.csdn.net/jinking01/article/details/108486601；该网址为cmakelist文件的详细介绍
cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

#显式定义使用SET指令构建自定义变量，比如:SET(HELLO_SRC main.c)就可以通过${HELLO_SRC}来引用这个自定义变量了
#编译类型为release，不是debug
set(CMAKE_BUILD_TYPE "Release")
#设置C++编译选项,使用C++11版本标准库； 如果使用noetic,需要使用C++14
set(CMAKE_CXX_FLAGS "-std=c++11")
#设置编译类型为Release时的编译选项：-03（最高级优化） -wall（开启警告） -g（开启调试）
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge 
  tf
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

#在指定目标之前，需要指定可以为所述目标找到资源的位置，特别是头文件和库
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#catkin_package()是一个由catkin提供的CMake宏。需要指定特定的catkin信息到编译系统，而这些信息又会被用于生成pkg-config和CMake文件。
#该函数必须在使用 add_library()或add_executable()声明任何targets之前调用。
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs #当前项目依赖的其他catkin项目
  DEPENDS EIGEN3 PCL #当前项目依赖的非catkin CMake项目
  INCLUDE_DIRS include #软件包导出的头文件路径
)

#指定编译，将cxx编译成可执行文件:(第一个参数为可执行文件名 其余参数为需要编译的文件)
add_executable(ascanRegistration src/scanRegistration.cpp)
#定义target依赖的其它target，确保在编译本target之前，其它的target已经被构建：（可执行文件名 依赖项）
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})