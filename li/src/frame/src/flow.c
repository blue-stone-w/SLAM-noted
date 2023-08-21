数据格式 {
  CloudData {点,点云,时间}
  IMUData {线速度,角速度,方向,四元数转换为矩阵函数,同步函数}
  GNSSData {时间,经纬高,转换为xyz函数,位姿初始化函数,同步函数}
  VelocityData {线速度,角速度,同步函数,坐标系转换函数}
  PoseData {4*4矩阵,转换为四元数}
}

订阅subscriber {ros订阅,缓存数据,读取数据函数}

data_pretreat {
  DataPretreatFlow构造函数 {
    CloudSubscriber构造函数 {
      msg_callback {
        转换消息格式并放入点云队列(自定义格式cloud_data)
      }
    }
    IMUSubscriber构造函数 {
      msg_callback {
        转换消息格式并放入imu队列(自定义格式)
      }
    }
    VelocitySubscriber构造函数 {
      msg_callback {
        转换消息格式并放入速度队列
      }
    }
    GNSSSubscriber构造函数 {
      msg_callback {
        转换消息格式并放入gnss队列
      }
    }
    TFListener构造函数 {传入子tf id和base tf id}
    CloudPublisher构造函数
    OdometryPublisher构造函数(gnss发布指针)
    用默认构造函数实例化DistortionAdjust
  }
  循环处理 {
    读取数据 { (函数声明在subscriber中)
      获取点云和未时间同步的imu速度gps
      根据点云的时间,通过插值时间同步imu速度gps
      有数据同步失败则返回false
    }
    初始化标定 {
      读取tf计算确定lidar和imu变换矩阵
    }
    用0初始化gnss的经纬高
    判断是否有足够的数据
    同步后的数据的时间相差大,则删除旧的数据并返回false
    畸变补偿 {
      把gnss的经纬高转换为xyz
      把imu四元数转换为旋转矩阵
      把速度转换为imu坐标系
      设定扫描周期和速度
      调整点云 { (匀速假设)
        旋转该帧所有点,使第一个点为x轴正方向
        遍历该帧所有点 { (每帧的时间戳是启始时刻和终止时刻的中值,因此以每帧的中间时刻作为基准值去畸变)
          计算点的原始方向和时间
          根据速度,角速度和时间得到旋转和平移放入gnss_pose_
          补偿畸变
        }
        旋转该帧所有点,使第一个点为原方向
      }
    }
    将去畸变的点云转换为PC2发布
    把gnss_pose_转换为nav_msgs::Odometry发布
  }
}

前端 {
  FrontEndFlow构造函数 {
    CloudSubscriber构造函数 {
      msg_callback {
        转换消息格式并放入点云队列(自定义格式cloud_data)
      }
    }
    OdometryPublisher构造函数(laser odom发布指针)
    FrontEnd构造函数 {
      前端初始化 {
        初始化参数
        初始化帧间匹配算法 {
          创建NDT匹配算法实例 {
            创建pcl::NormalDistributionsTransform
            传入NDT参数并设置
          }
        }
        初始化local_map的滤波方式 {
          创建体素滤波实例 {
            pcl::VoxelGrid
            传入NDT参数并设置
          }
        }
        初始化frame的滤波方式 {
          创建无滤波实例 {          }
        } 
      }
    }
  }
  循环处理 {
    读取数据 {读取点云数据}
    如果有点云数据则循环 {
      更新laserodom {
        如果需要初始化odom {
          初始化位姿为单位矩阵
          移除nan点
          当前frame滤波
          用初始化的位姿 初始化step_pose、上一位姿、预测位姿、上一关键帧位姿
          把关键帧加入局部地图,删除旧的关键帧以维持局部地图体积
          把局部地图转换到???坐标系下
          更新ndt匹配的目标点云 {
            局部地图的关键帧数量较少时 {
              把局部地图作为ndt匹配的目标点云
            } 否则 {
              对局部地图滤波,把滤波后的局部地图作为ndt匹配的目标点云
            }
          }
          NDT匹配 {
            调用pcl的ndt匹配得到位姿
          }
          更新step_pose、上一位姿、预测位姿(根据匀速假设预测)
          根据距离判断是否生成新的关键帧 {
            把关键帧加入局部地图,删除旧的关键帧以维持局部地图体积
            把局部地图转换到???坐标系下
            更新ndt匹配的目标点云 {
              局部地图的关键帧数量较少时 {
                把局部地图作为ndt匹配的目标点云
              } 否则 {
                对局部地图滤波,把滤波后的局部地图作为ndt匹配的目标点云
              }
            }
          }
        }
        移除nan点
        当前frame滤波
        把关键帧加入局部地图,删除旧的关键帧以维持局部地图体积
        把局部地图转换到???坐标系下
        更新ndt匹配的目标点云 {
          局部地图的关键帧数量较少时 {
            把局部地图作为ndt匹配的目标点云
          } 否则 {
            对局部地图滤波,把滤波后的局部地图作为ndt匹配的目标点云
          }
        }
        NDT匹配 {
          调用pcl的ndt匹配得到位姿
        }
        更新step_pose、上一位姿、预测位姿(根据匀速假设预测)
        根据距离判断是否生成新的关键帧 {
          把关键帧加入局部地图,删除旧的关键帧以维持局部地图体积
          把局部地图转换到???坐标系下
          更新ndt匹配的目标点云 {
            局部地图的关键帧数量较少时 {
              把局部地图作为ndt匹配的目标点云
            } 否则 {
              对局部地图滤波,把滤波后的局部地图作为ndt匹配的目标点云
            }
          }
        }
      }
      PublishData {
        nav_msgs::Odometry(2 frameid,transform,time)
      }
    }
  }
}

后端 {
  ros::ServiceServer:optimize_map_callback {  }
  BackEndFlow构造函数 {
    CloudSubscriber构造函数:已同步点云
    gnss(OdometrySubscriber):已同步gnss
    LO(OdometrySubscriber):laser_odom,前端发布
    loop(LoopPoseSubscriber)

    transformed_odom<OdometryPublisher>"/transformed_odom", "/map", "/lidar", 100);
    key_frame<KeyFramePublisher>("/key_frame", "/map", 100);
    key_gnss<KeyFramePublisher>("/key_gnss", "/map", 100);
    key_frames<KeyFramesPublisher>("/optimized_key_frames", "/map", 100);

    BackEnd构造函数 {
      初始化后端 {
        初始化参数:关键帧距离
        初始化图优化器 {
          图优化器构造函数 {
            构建稀疏优化器,采用LM迭代
          }
          图优化配置 {
            使用的优化来源:gnss,loop
            优化的间隔:关键帧,gnss,回环
            初始化odom、回环和gnss噪声
          }
        }
        初始化数据路径 {关键帧和轨迹}
      }
    }
  }
  循环处理 {
    读取订阅的4种数据
    如果有回环信息则插入回环 {
      添加边 {
        用噪声计算信息矩阵
        得到两个顶点
        得到边:测量值,信息矩阵,对应的顶点
        将边加入优化器
        添加核函数 {
          选择核函数的类型
          设置核的大小
          为边添加核函数
        }
      }
    }
    如果有数据则循环 {
      如果数据时刻不合适则下一轮循环
      更新后端 {
        如果未初始化 {
          旋转LO至gnss坐标系
        } 
        重置参数为没有新的关键帧和优化
        根据距离判断是否为关键帧 {
          把关键帧的点云存储进硬盘
          得到当前关键帧和关键gnss
          保存位姿为txt文件
          添加边和节点 {
            添加关键帧节点 {
              设置节点的id和估计初值
              设置是否fix(若该位姿为真值,设为false)
              将节点加入优化器
            }
            添加LO边 {
              计算相对位姿
              添加边
            }
            添加gnss先验边 {
              计算xyz
              计算信息矩阵
              计算节点
              声明并定义先验边
              将边加入优化器
              添加核函数
            }
          }
          如果间隔的gnss关键帧回环足够多则优化 {
            执行g2o相关函数
            执行优化函数
            把节点中保存的估计的位姿取出并保存
          }
        }
      }
    }
    发布数据 {
      发布transformed_odom
      若有关键帧则发布关键帧和关键gnss
      获取已优化的所有关键帧并发布
    }
    如果需要优化后的地图则强制优化 {
      执行优化函数
      把节点中保存的估计的位姿取出并保存
    }
  }
}

回环 {
  LoopClosingFlow构造函数 {
    订阅关键帧<KeyFrameSubscriber>
    订阅关键gnss<KeyFrameSubscriber>
    发布回环<LoopPosePublisher>构造函数 {
      构造发布
    }
    闭环<LoopClosing>构造函数 {
      初始化闭环 {
        初始化参数:距离间隔,匹配分数,回环帧和普通关键帧的检测间隔
        初始化数据保存路径
        初始化匹配 {          NDT匹配        }
        初始化map滤波 { 体素滤波       }
        初始化scan滤波
      }
    }
  }
  循环处理 {
    读取关键帧和关键gnss
    如果有数据则循环 {
      如果数据时刻不合适则下一轮循环
      寻找最近的关键帧 {
        间隔足够的帧
        遍历寻找最近帧和最近距离
        无法构成滑窗则返回
        最近帧太远 {
          重新设置闭环跳过的关键帧数量(在最近距离范围内不检测闭环)
        }
      }
      点云配准 {
        生成地图 {
          计算关键帧在gnss坐标系下的位姿
          根据位姿变换点云把点云加入地图
          地图点云滤波
        }
        生成当前帧 {
          使用gnss坐标
          滤波该帧点云
        }
        匹配函数
        计算相对位姿
        判断回环是否有效
      }
      发布回环的相对位姿
    }
  }
}

可视化 {
  保存地图服务
  _viewer_flow_ptr<ViewerFlow>构造函数 {
    订阅点云、关键帧、transformed_odom、已优化的关键帧
    optimized_odom_pub<OdometryPublisher>
    current_scan_pub<CloudPublisher>
    global_map<CloudPublisher>
    local_map<CloudPublisher>
    Viewer构造函数 {
      初始化显示 {
        初始化参数 {局部帧的数量}
        初始化数据路径 {}
        初始化帧、局部帧、全局帧的滤波算法
      }
    }
  }
  循环处理 {
    读取数据 {
      
    }
  }
}