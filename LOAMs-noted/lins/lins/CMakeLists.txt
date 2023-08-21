cmake_minimum_required(VERSION 2.8.3)
project(lins)
#设置C++编译选项，使用C++11版本标准库；-03（最高级优化）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# The QUIET option disables informational messages, including those indicating that the package cannot be found if it is not REQUIRED. 
# The REQUIRED option stops processing with an error message if the package cannot be found.
# A package-specific list of required components may be listed after the COMPONENTS keyword. If any of these components are not able to be satisfied, the package overall is considered to be not found.
# to ensure that all needed packages are exit
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cloud_msgs
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED)

# catkin_package()是catkin提供的CMake宏，用于为catkin提供构建、生成pkg-config和CMake文件所需要的信息。
# INCLUDE_DIRS - 声明给其它package的include路径
# LIBRARIES - 声明给其它package的库
# CATKIN_DEPENDS - 本包依赖的catkin package
# DEPENDS - 本包依赖的非catkin package
# CFG_EXTRAS - 其它配置参数
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS cloud_msgs
    DEPENDS PCL
)


# 相当于g++选项中的-I参数的作用，也相当于环境变量中增加路径到CPLUS_INCLUDE_PATH变量的作用
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# 添加需要链接的库文件目录;相当于g++命令的-L选项的作用，也相当于环境变量中增加LD_LIBRARY_PATH的路径的作用
# eigen is included in pcl, therefor here isn't eigen
link_directories(
    include
    ${OpenCV_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)

# (APPEND <list_element> [<element> ...])
list(APPEND SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/lib/parameters.cpp
)

list(APPEND LINS_FILES
    ${PROJECT_SOURCE_DIR}/src/lins_fusion_node.cpp
    ${PROJECT_SOURCE_DIR}/src/lib/Estimator.cpp
)

list(APPEND LINK_LIBS 
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARY_DIRS}
)

add_executable(lins_fusion_node ${LINS_FILES} ${SOURCE_FILES})
target_link_libraries(lins_fusion_node ${LINK_LIBS})

add_executable(image_projection_node src/image_projection_node.cpp ${SOURCE_FILES})
add_dependencies(image_projection_node ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(image_projection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lidar_mapping_node src/lidar_mapping_node.cpp ${SOURCE_FILES})
target_link_libraries(lidar_mapping_node ${catkin_LIBRARIES} ${LINK_LIBS} gtsam)

add_executable(transform_fusion_node src/transform_fusion_node.cpp ${SOURCE_FILES})
target_link_libraries(transform_fusion_node ${LINK_LIBS})