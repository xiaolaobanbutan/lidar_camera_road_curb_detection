#include "lidar_camera_road_curb_detection/feature.hpp"



namespace CurbDetection {

// 初始化参数
Feature_Point::Feature_Point(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                     std::vector<IndexRange> scanIndices,
                const featurePointsMsg &fpMsg

                ){
    _incloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _incloud = incloud;
    _scanindices = scanIndices;


    _points_ring_list.resize(fpMsg.nScanRings);

    for (size_t i = 0; i < _points_ring_list.size(); ++i) {
      _points_ring_list[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }

    _feature_points_list.resize(fpMsg.nScanRings);

    // 初始化
    for (size_t i = 0; i < _feature_points_list.size(); ++i) {
      _feature_points_list[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }

    for (int i = 0; i < incloud->points.size(); ++i) {
      size_t j = incloud->points[i].intensity;
      _points_ring_list[j]->points.push_back(incloud->points[i]);
    }

    _Height_Region = fpMsg.heightRegion; // 高度取点云范围
    _Height_SigmaThres = fpMsg.heightSigmaThre;   // 高度方差阈值
    _Height_MaxThres = fpMsg.heightMaxThres; // 高度差最大值
    _Height_MinThres = fpMsg.heightMinThres; // 高度差最小值
    _Curvature_Thres = fpMsg.curvatureThres; // 曲率阈值
    _Curvature_Region = fpMsg.curvatureRegion; // 曲率取点云范围
    _Distance_HorizonThres = fpMsg.distanceHorizonThres; // 水平面的距离阈值
    _Distance_VerticalThres = fpMsg.distanceVerticalThres; // 垂直面的距离阈值
    _Angular_Res = fpMsg.angularRes;  // 雷达的角分辨率
    _use_verticle = fpMsg.useVerticle;
    _use_horizon = fpMsg.useHorizon;
    _use_anglehorizon = fpMsg.useAngleHorizon;
    _Test_Region = fpMsg.testRegion;
    _Test_Thres = fpMsg.testThres;
}


/**
 * @brief 计算编号index的点云在region范围内最左边与最右边点的水平角度差
 *
 * @param index
 * @param region
 * @return float
 */
float Feature_Point::compute_Horizon_Diff(int index, int region) {
  float ori_1 = std::atan2(_incloud->points[index - region].y,
                           _incloud->points[index - region].x) * 180 / M_PI;
  float ori_2 = std::atan2(_incloud->points[index + region].y,
                           _incloud->points[index + region].x) * 180 / M_PI;

  ori_1 = ori_1 < 0 ? ori_1 + 360 : ori_1;
  ori_2 = ori_2 < 0 ? ori_2 + 360 : ori_2;
  return abs(ori_1 - ori_2);
}


/**
 * @brief 计算编号index的点云在region范围内最左边与最右边点的XY平面上的角度差
 *
 * @return float
 */
float Feature_Point::compute_Horizon_Diff_32c(int ns,int index, int region) {
  float ori_1 = std::atan2(_points_ring_list[ns]->points[index - region].y,
                           _points_ring_list[ns]->points[index - region].x) * 180 / M_PI;
  float ori_2 = std::atan2(_points_ring_list[ns]->points[index + region].y,
                           _points_ring_list[ns]->points[index + region].x) * 180 / M_PI;

  ori_1 = ori_1 < 0 ? ori_1 + 360 : ori_1;
  ori_2 = ori_2 < 0 ? ori_2 + 360 : ori_2;
  return abs(ori_1 - ori_2);
}



/**
 * @brief 计算同一扫描线其邻域内高度的标准差
 *
 * @param index
 * @param region
 * @return float
 */
float Feature_Point::compute_Height_Sigma(int index, int region) {
  //高度方差
  float meanZ = _incloud->points[index].z;

  for (int j = 1; j <= region; j++) {
    meanZ += _incloud->points[index + j].z + _incloud->points[index - j].z;
  }

  meanZ = meanZ / (1 + 2 * region);

  float sigma_height = pow(_incloud->points[index].z - meanZ, 2);

  for (int j = 1; j <= region; j++) {
    sigma_height += pow(_incloud->points[index + j].z - meanZ, 2);
    sigma_height += pow(_incloud->points[index - j].z - meanZ, 2);
  }

  sigma_height = sqrt(sigma_height / (1 + 2 * region));

  return sigma_height;
}

// /**
//  * @brief 计算同一扫描线其邻域内高度的标准差
//  *
//  * @param ns
//  * @param index
//  * @param region
//  * @return float
//  */
// float Feature_Point::compute_Height_Sigma_32c(int ns, int index, int region) {
//   //高度方差
//   float meanZ = _points_ring_list[ns]->points[index].z;

//   for (int j = 1; j <= region; j++) {
//     meanZ += _points_ring_list[ns]->points[index + j].z + _points_ring_list[ns]->points[index - j].z;
//   }

//   meanZ = meanZ / (1 + 2 * region);

//   float sigma_height = pow(_points_ring_list[ns]->points[index].z - meanZ, 2);

//   for (int j = 1; j <= region; j++) {
//     sigma_height += pow(_points_ring_list[ns]->points[index + j].z - meanZ, 2);
//     sigma_height += pow(_points_ring_list[ns]->points[index - j].z - meanZ, 2);
//   }

//   sigma_height = sqrt(sigma_height / (1 + 2 * region));

//   return sigma_height;
// }


/**
 * @brief 计算给定点及其邻域内高度的最大值
 *
 * @param j
 * @return float
 */
float Feature_Point::Region_MaxZ(int j) {
  float max_z = _incloud->points[j].z;

  for (int k = j - _Height_Region; k <= j + _Height_Region; ++k) {
    if (max_z < _incloud->points[k].z) {
      max_z = _incloud->points[k].z;
    }
  }

  return max_z;
}

/**
 * @brief 计算给定点及其邻域内高度的最大值
 *
 * @param j
 * @return float
 */
float Feature_Point::Region_MinZ(int j) {
  float min_z = _incloud->points[j].z;

  for (int k = j - _Height_Region; k <= j + _Height_Region; ++k) {
    if (min_z > _incloud->points[k].z) {
      min_z = _incloud->points[k].z;
    }
  }
  return min_z;
}


/**
 * @brief 计算给定点及其邻域内 Z 的最大值
 *
 * @param ns
 * @param j
 * @param region
 * @return float
 */
float Feature_Point::Region_MaxZ_32c(int ns,int j, int region) {
  float max_z = _points_ring_list[ns]->points[j].z;

  for (int k = j - region; k <= j + region; ++k) {
    if (max_z < _points_ring_list[ns]->points[k].z) {
      max_z = _points_ring_list[ns]->points[k].z;
    }
  }

  return max_z;
}

/**
 * @brief 计算给定点及其邻域内 Z 的最小值
 *
 * @param ns
 * @param j
 * @param region
 * @return float
 */
float Feature_Point::Region_MinZ_32c(int ns,int j, int region) {
  float min_z = _points_ring_list[ns]->points[j].z;

  for (int k = j - region; k <= j + region; ++k) {
    if (min_z > _points_ring_list[ns]->points[k].z) {
      min_z = _points_ring_list[ns]->points[k].z;
    }
  }
  return min_z;
}



/**
 * @brief 计算点到原点的距离
 *
 * @param p
 * @return float
 */
float Feature_Point::calc_Point_Distance(const pcl::PointXYZI &p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * @brief 计算两个向量的交集
 *
 * @param v1
 * @param v2
 * @return std::vector<int>
 */
std::vector<int> Feature_Point::Vectors_Intersection(std::vector<int> v1,
                                                     std::vector<int> v2) {
  std::vector<int> v;
  sort(v1.begin(), v1.end());
  sort(v2.begin(), v2.end());
  set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                   back_inserter(v));

  return v;
}



/**
 * @brief 使用 pcl::ExtractIndices 提取点云中指定索引的点
 *
 * @param incloud
 * @param outCloud
 * @param indices
 * @param setNeg
 */
void Feature_Point::extract_Points(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                                    boost::shared_ptr<std::vector<int>> indices,
                                    bool setNeg) {
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setNegative(setNeg);
  extract.setInputCloud(incloud);
  extract.setIndices(indices);
  extract.filter(*outCloud);
}

/**
 * @brief
 *
 * @param feature_points
 */
void Feature_Point::extract_Features(pcl::PointCloud<pcl::PointXYZI>::Ptr& feature_points) {


  double start = ros::Time::now().toSec();


  size_t nScans = _scanindices.size();

  float angleRegionThres = _Angular_Res * 4;

  float NSCAN = 64;

  for (size_t i = 0; i < nScans; i++) {

    float nScans_proportion = (NSCAN - i) / NSCAN;

    // i 是雷达线数
    size_t scanStartIdx = _scanindices[i].first; // 从第i圈第一个开始
    size_t scanEndIdx = _scanindices[i].second; // 到第i圈最后一个结束

    // 跳过点数小于100的scan
    if (scanEndIdx <= scanStartIdx + 100) {
      continue;
    }

    //提取高度特征点
    for (int k = scanStartIdx + _Height_Region; k <= scanEndIdx - _Height_Region; ++k) {

      // 防止出现两条扫描线扫描的物体不在同一垂直面或者水平面
      if (compute_Horizon_Diff(k, _Height_Region) > (_Height_Region * 2) * angleRegionThres) {
        continue;
      }

      float heightDiff = Region_MaxZ(k) - Region_MinZ(k);

      // 高度方差点
      float sigma_height = compute_Height_Sigma(k, _Height_Region);

      // 方差和高度差值都满足条件视为候选curb点
      // 单位m

      if (heightDiff >= _Height_MinThres &&
          heightDiff <= _Height_MaxThres &&
          sigma_height >= _Height_SigmaThres) {
        _HeightPointsIndex.push_back(k);
      }

    }


    // 提取平滑特征点
    float pointWeight = -2 * _Curvature_Region;

    for (size_t k = scanStartIdx + _Curvature_Region;
         k <= scanEndIdx - _Curvature_Region; k++) {
      if (compute_Horizon_Diff(k, _Curvature_Region) > (_Curvature_Region * 2) * angleRegionThres) {
        continue;
      }

      float diffX = pointWeight * _incloud->points[k].x;
      float diffY = pointWeight * _incloud->points[k].y;
      float diffZ = pointWeight * _incloud->points[k].z;

      for (int j = 1; j <= _Curvature_Region; j++) {
        diffX += _incloud->points[k + j].x + _incloud->points[k - j].x;
        diffY += _incloud->points[k + j].y + _incloud->points[k - j].y;
        diffZ += _incloud->points[k + j].z + _incloud->points[k - j].z;
      }

      float curvatureValue =
          sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) /
          (_Curvature_Region * calc_Point_Distance(_incloud->points[k]));
      if (  curvatureValue >  _Curvature_Thres) {
        _CurvaturePointsIndex.push_back(k);
      }
    }


    // 平面距离特征点，未使用
    for (int k = scanStartIdx + 1; k <= scanEndIdx; ++k) {
      if (compute_Horizon_Diff(k, 1) > (2 * angleRegionThres)) {
        continue;
      }

      // 间隔距离 d = θ × r ，θ为间隔角，r为半径
      float distanceHorizonThreshold =
          sqrt(pow(_incloud->points[k].x, 2) + pow(_incloud->points[k].y, 2)) *M_PI * _Angular_Res / 180;

      // 两点之间水平距离
      float distancePre =
          sqrt(pow(_incloud->points[k].x - _incloud->points[k - 1].x, 2) + pow(_incloud->points[k].y - _incloud->points[k - 1].y, 2));

      if (distancePre < distanceHorizonThreshold * _Distance_HorizonThres) {
        _DistanceHorizonPointsIndex.push_back(k);
      }
    }


    // 等边三角形法
    for (int k = scanStartIdx + _Test_Region; k <= scanEndIdx - _Test_Region; ++k) {
      if (compute_Horizon_Diff(k, _Test_Region) > (_Test_Region * 2 * angleRegionThres)) {
        continue;
      }

      float le =
          sqrt(pow(_incloud->points[k].x - _incloud->points[k - _Test_Region].x, 2) + pow(_incloud->points[k].y - _incloud->points[k - _Test_Region].y, 2));

      // 两点之间水平距离
      float ri =
          sqrt(pow(_incloud->points[k].x - _incloud->points[k + _Test_Region].x, 2) + pow(_incloud->points[k].y - _incloud->points[k + _Test_Region].y, 2));

      if (abs(le - ri) > _Test_Region * _Test_Thres/(i+1)) {
        _TestIndex.push_back(k);
      }
    }



    // //垂直距离特征，未使用
    // for (int k = scanStartIdx + 1; k <= scanEndIdx; ++k) {
    //   if (compute_Horizon_Diff(k, 1) > (1 * 2) * angleRegionThres) {
    //     continue;
    //   }

    //   float angleVertical =
    //       std::atan(_incloud->points[k].z / sqrt(pow(_incloud->points[k].x, 2) +
    //                                            pow(_incloud->points[k].y, 2)));

    //   float distanceHorizonThreshold =
    //       fabs(sin(angleVertical)) *
    //       sqrt(pow(_incloud->points[k].x, 2) + pow(_incloud->points[k].y, 2)) *
    //       M_PI * _Angular_Res / 180;

    //   float distancePre = abs(_incloud->points[k].z - _incloud->points[k - 1].z);
    //   if (distancePre > distanceHorizonThreshold * _Distance_VerticalThres) {
    //     _DistanceVerticlePointsIndex.push_back(k);
    //   }
    // }


  }


  //提取高度特征点和平滑特征点
  _INDEX = Vectors_Intersection(_HeightPointsIndex, _CurvaturePointsIndex);

  // 是否使用平面距离特征点
  if (_use_horizon) {
    _INDEX = Vectors_Intersection(_INDEX, _DistanceHorizonPointsIndex);
  }

  // 是否使用平面距离特征点
  if (_use_anglehorizon) {
    _INDEX = Vectors_Intersection(_INDEX,_TestIndex);
  }

  // // 是否使用垂直距离特征点
  // if (_use_verticle) {
  //   _INDEX = Vectors_Intersection(_INDEX, _DistanceVerticlePointsIndex);
  // }

  // // 测试
  // //提取高度特征点和平滑特征点
  // _INDEX = _TestIndex;
  // 最终得到的特征点索引存储在 _INDEX 中
  extract_Points(_incloud, feature_points, boost::make_shared<std::vector<int>>(_INDEX));

  double end = ros::Time::now().toSec();

  // std::cout << "\033[1m\033[36m" << "获取特征点消耗时间 = " << end - start << " 秒. " << endl;
}


void Feature_Point::extract_Features_XYZIRT(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vector,pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points) {


  double start = ros::Time::now().toSec();
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _points;

  size_t nScans = cloud_vector.size();

  _HeightPointsIndex_XYZIRT.resize(nScans);
  _CurvaturePointsIndex_XYZIRT.resize(nScans);
  _TestIndex_XYZIRT.resize(nScans);
  _INDEX_XYZIRT.resize(nScans);

  _points.resize(nScans);
  for (size_t i = 0; i < nScans; ++i) {
      _points[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
  }


  float angleRegionThres = _Angular_Res * 4;

  float NSCAN = 32;

  for (size_t i = 0; i < nScans; i++) {

    float nScans_proportion = (NSCAN - i) / NSCAN;

    if (cloud_vector[i]->size()<=100) {
      continue;}

    //提取高度特征点
    for (int k = 0 + _Height_Region; k <= cloud_vector[i]->size() - _Height_Region; ++k) {

      float ori_1 = std::atan2(cloud_vector[i]->points[k - _Height_Region].y,cloud_vector[i]->points[k - _Height_Region].x) * 180 / M_PI;
      float ori_2 = std::atan2(cloud_vector[i]->points[k - _Height_Region].y,cloud_vector[i]->points[k + _Height_Region].x) * 180 / M_PI;
      ori_1 = ori_1 < 0 ? ori_1 + 360 : ori_1;
      ori_2 = ori_2 < 0 ? ori_2 + 360 : ori_2;
      float angleDiff = abs(ori_1 - ori_2);

      // 防止出现两条扫描线扫描的物体不在同一垂直面或者水平面
      if (angleDiff > (_Height_Region * 2) * angleRegionThres) {
        continue;}

      // 计算领域内高度的最大值和最小值
      float max_z = cloud_vector[i]->points[k].z;
      float min_z = cloud_vector[i]->points[k].z;
      for (int j = k - _Height_Region; j <= k + _Height_Region; ++j) {
        if (max_z < cloud_vector[i]->points[j].z)
          max_z = cloud_vector[i]->points[j].z;
        if (min_z > cloud_vector[i]->points[j].z)
          min_z = cloud_vector[i]->points[j].z;
        }
      float heightDiff = max_z - min_z;

      // 高度方差
      float meanZ = 0.0f;
      for (int j = k - _Height_Region; j <= k + _Height_Region; ++j) {
        meanZ += cloud_vector[i]->points[j].z;
      }
      meanZ = meanZ / (1 + 2 * _Height_Region);

      float sigma_height = 0.0f;
      for (int j = k - _Height_Region; j <= k + _Height_Region; ++j) {
        sigma_height += pow(cloud_vector[i]->points[j].z - meanZ, 2);
      }
      sigma_height = sqrt(sigma_height / (1 + 2 * _Height_Region));


      // 方差和高度差值都满足条件视为候选curb点
      // 单位m

      if (heightDiff >= _Height_MinThres &&
          heightDiff <= _Height_MaxThres &&
          sigma_height >= _Height_SigmaThres) {
        _HeightPointsIndex_XYZIRT[i].push_back(k);
      }
    }

    // 提取平滑特征点
    float pointWeight = -2 * _Curvature_Region;
    for (int k = 0 + _Curvature_Region; k <= cloud_vector[i]->size() - _Curvature_Region; ++k) {

          float ori_1 = std::atan2(cloud_vector[i]->points[k - _Curvature_Region].y,cloud_vector[i]->points[k - _Curvature_Region].x) * 180 / M_PI;
          float ori_2 = std::atan2(cloud_vector[i]->points[k - _Curvature_Region].y,cloud_vector[i]->points[k + _Curvature_Region].x) * 180 / M_PI;
          ori_1 = ori_1 < 0? ori_1 + 360 : ori_1;
          ori_2 = ori_2 < 0? ori_2 + 360 : ori_2;
          float angleDiff = abs(ori_1 - ori_2);
          // 防止出现两条扫描线扫描的物体不在同一垂直面或者水平面
          if (angleDiff > (_Curvature_Region * 2) * angleRegionThres) {
            continue;}



          float diffX = pointWeight * cloud_vector[i]->points[k].x;
          float diffY = pointWeight * cloud_vector[i]->points[k].y;
          float diffZ = pointWeight * cloud_vector[i]->points[k].z;
          for (int j = 1; j <= _Curvature_Region; j++) {
            diffX += cloud_vector[i]->points[k + j].x + cloud_vector[i]->points[k - j].x;
            diffY += cloud_vector[i]->points[k + j].y + cloud_vector[i]->points[k - j].y;
            diffZ += cloud_vector[i]->points[k + j].z + cloud_vector[i]->points[k - j].z;
          }
          float curvatureValue =
              sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) /
              (_Curvature_Region * calc_Point_Distance(cloud_vector[i]->points[k]));
          if (  curvatureValue >  _Curvature_Thres) {
            _CurvaturePointsIndex_XYZIRT[i].push_back(k);
          }
    }


    // // 平面距离特征点
    // for (int k = scanStartIdx + 1; k <= scanEndIdx; ++k) {
    //   if (compute_Horizon_Diff(k, 1) > (2 * angleRegionThres)) {
    //     continue;
    //   }

    //   // 间隔距离 d = θ × r ，θ为间隔角，r为半径
    //   float distanceHorizonThreshold =
    //       sqrt(pow(_incloud->points[k].x, 2) + pow(_incloud->points[k].y, 2)) *M_PI * _Angular_Res / 180;

    //   // 两点之间水平距离
    //   float distancePre =
    //       sqrt(pow(_incloud->points[k].x - _incloud->points[k - 1].x, 2) + pow(_incloud->points[k].y - _incloud->points[k - 1].y, 2));

    //   if (distancePre < distanceHorizonThreshold * _Distance_HorizonThres) {
    //     _DistanceHorizonPointsIndex.push_back(k);
    //   }
    // }

    // 三角形方法
    for(int k = 0 + _Test_Region; k <= cloud_vector[i]->size() - _Test_Region; ++k) {
          float ori_1 = std::atan2(cloud_vector[i]->points[k - _Test_Region].y,cloud_vector[i]->points[k - _Test_Region].x) * 180 / M_PI;
          float ori_2 = std::atan2(cloud_vector[i]->points[k - _Test_Region].y,cloud_vector[i]->points[k + _Test_Region].x) * 180 / M_PI;
          ori_1 = ori_1 < 0? ori_1 + 360 : ori_1;
          ori_2 = ori_2 < 0? ori_2 + 360 : ori_2;
          float angleDiff = abs(ori_1 - ori_2);
          angleDiff = std::min(angleDiff, 360.0f - angleDiff);
          // 防止出现两条扫描线扫描的物体不在同一垂直面或者水平面
          if (angleDiff > (_Test_Region * 2) * angleRegionThres) {
            continue;}

      float le =
          sqrt(pow(cloud_vector[i]->points[k].x - cloud_vector[i]->points[k - _Test_Region].x, 2) +
          pow(cloud_vector[i]->points[k].y - cloud_vector[i]->points[k - _Test_Region].y, 2));
      // 两点之间水平距离
      float ri =
          sqrt(pow(cloud_vector[i]->points[k].x - cloud_vector[i]->points[k + _Test_Region].x, 2) +
          pow(cloud_vector[i]->points[k].y - cloud_vector[i]->points[k + _Test_Region].y, 2));

      if (abs(le - ri) > _Test_Region * _Test_Thres/(i+1)) {
        _TestIndex_XYZIRT[i].push_back(k);
      }

    }



    // //垂直距离特征，未使用
    // for (int k = scanStartIdx + 1; k <= scanEndIdx; ++k) {
    //   if (compute_Horizon_Diff(k, 1) > (1 * 2) * angleRegionThres) {
    //     continue;
    //   }

    //   float angleVertical =
    //       std::atan(_incloud->points[k].z / sqrt(pow(_incloud->points[k].x, 2) +
    //                                            pow(_incloud->points[k].y, 2)));

    //   float distanceHorizonThreshold =
    //       fabs(sin(angleVertical)) *
    //       sqrt(pow(_incloud->points[k].x, 2) + pow(_incloud->points[k].y, 2)) *
    //       M_PI * _Angular_Res / 180;

    //   float distancePre = abs(_incloud->points[k].z - _incloud->points[k - 1].z);
    //   if (distancePre > distanceHorizonThreshold * _Distance_VerticalThres) {
    //     _DistanceVerticlePointsIndex.push_back(k);
    //   }
    // }


  }


  //提取高度特征点和平滑特征点
  for (size_t i = 0; i < nScans; i++) {
    _INDEX_XYZIRT[i] = Vectors_Intersection(_HeightPointsIndex_XYZIRT[i], _CurvaturePointsIndex_XYZIRT[i]);
    // 是否使用平面距离特征点
    if (_use_horizon) {
      _INDEX_XYZIRT[i] = Vectors_Intersection(_INDEX_XYZIRT[i], _DistanceHorizonPointsIndex);

    }
    // 是否使用三角形平面距离特征点
    if (_use_anglehorizon) {
      _INDEX_XYZIRT[i] = Vectors_Intersection(_INDEX_XYZIRT[i],_TestIndex_XYZIRT[i]);
    }
    // // 是否使用垂直距离特征点
    // if (_use_verticle) {
    //   _INDEX_XYZIRT[i] = Vectors_Intersection(_INDEX_XYZIRT[i], _DistanceVerticlePointsIndex);
    // }


    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setIndices(boost::make_shared<std::vector<int>>(_INDEX_XYZIRT[i]));
    extract.setInputCloud(cloud_vector[i]);
    extract.filter(*_points[i]);
  }

  for(size_t i = 0; i < nScans; i++) {
    *feature_points += *_points[i];
  }
}


/**
 * @brief
 *
 * @param feature_points
 */
// void Feature_Point::extract_Features_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points) {

//   size_t nScans = _points_ring_list.size();

//   float angleRegionThres = _Angular_Res * 4;
//   _Height_Points_Index.resize(nScans);
//   _Curvature_PointsIndex.resize(nScans);
//   _Distance_HorizonPointsIndex.resize(nScans);
//   _Distance_VerticlePointsIndex.resize(nScans);
//   _Index.resize(nScans);

//   for (size_t i = 0; i < nScans; i++) {

//     // 跳过点数小于100的scan
//     if (_points_ring_list[i]->size() <= 100) {
//       continue;
//     }

//     //提取高度特征点
//     for (int k = 0 + _Height_Region; k <= _points_ring_list[i]->size() - _Height_Region; ++k) {

//       // 防止扫描线在 点k 的范围内扫描的点相差太大
//       // compute_Horizon_Diff_32c计算编号index的点云在region范围内最左边与最右边点的XY平面上的角度差
//       // 从理论上来说在同一个水平面，注意是水平面上的角度差应该是 角分辨率Angular_Res × 范围region(_Height_Region * 2)
//       // 这里给了一定的阈值 angleRegionThres = _Angular_Res * 4
//       if (compute_Horizon_Diff_32c(i, k, _Height_Region) > (_Height_Region * 2) * angleRegionThres) {
//         continue;
//         }

//       // 计算 k 范围内高度差最大值
//       float heightDiff = Region_MaxZ_32c(i, k, _Height_Region) - Region_MinZ_32c(i, k, _Height_Region);

//       // 计算 k 范围内高度方差
//       float sigma_height = compute_Height_Sigma_32c(i, k, _Height_Region);

//       //方差和高度差值都满足条件视为候选curb点
//       if (heightDiff >= _Height_MinThres &&
//           heightDiff <= _Height_MaxThres &&
//           sigma_height >= _Height_SigmaThres) {
//         _Height_Points_Index[i].push_back(k);
//       }

//     }


//     // 提取平滑特征点,公式：从中心点到范围内所有点构成的向量，相加，除以范围内点数，然后再除以该中心点的模
//     float pointWeight = -2 * _Curvature_Region;

//     for (size_t k = 0 + _Curvature_Region; k <= _points_ring_list[i]->size() - _Curvature_Region; k++) {

//       // 防止扫描线在 点k 的范围内扫描的点相差太大
//       if (compute_Horizon_Diff_32c(i, k, _Curvature_Region) > (_Curvature_Region * 2) * angleRegionThres) {
//         continue;
//         }

//       float diffX = pointWeight * _points_ring_list[i]->points[k].x;
//       float diffY = pointWeight * _points_ring_list[i]->points[k].y;
//       float diffZ = pointWeight * _points_ring_list[i]->points[k].z;

//       for (int j = 1; j <= _Curvature_Region; j++) {
//         diffX += _points_ring_list[i]->points[k + j].x + _points_ring_list[i]->points[k - j].x;
//         diffY += _points_ring_list[i]->points[k + j].y + _points_ring_list[i]->points[k - j].y;
//         diffZ += _points_ring_list[i]->points[k + j].z + _points_ring_list[i]->points[k - j].z;
//       }

//       float curvatureValue =
//           sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) /
//           (_Curvature_Region * calc_Point_Distance(_points_ring_list[i]->points[k]));
//       if (curvatureValue > _Curvature_Thres) {
//         _Curvature_PointsIndex[i].push_back(k);
//       }
//     }


//     // 平面距离特征点,点k和右边点之间的距离
//     for (int k = 0 + 1; k <= _points_ring_list[i]->size(); ++k) {
//       if (compute_Horizon_Diff_32c(i, k, 1) > (2 * angleRegionThres)) {
//         continue;
//       }

//       // 间隔距离 d = θ × r ，θ为间隔角，r为半径
//       float distanceHorizonThreshold =
//           sqrt(pow(_points_ring_list[i]->points[k].x, 2) + pow(_points_ring_list[i]->points[k].y, 2)) *M_PI * _Angular_Res / 180;

//       // 两点之间水平距离
//       float distancePre =
//           sqrt(pow(_points_ring_list[i]->points[k].x - _points_ring_list[i]->points[k - 1].x, 2) + pow(_points_ring_list[i]->points[k].y - _points_ring_list[i]->points[k - 1].y, 2));

//       if (distancePre < distanceHorizonThreshold * _Distance_HorizonThres) {
//         _Distance_HorizonPointsIndex[i].push_back(k);
//       }
//     }


//     //垂直距离特征
//     for (int k = 0 + 1; k <= _points_ring_list[i]->size(); ++k) {
//       if (compute_Horizon_Diff_32c(i, k, 1) > (1 * 2) * angleRegionThres) {
//         continue;
//       }

//       // z轴和xy平面的角度
//       float angleVertical =
//           std::atan(_points_ring_list[i]->points[k].z / sqrt(pow(_points_ring_list[i]->points[k].x, 2) +
//                                                pow(_points_ring_list[i]->points[k].y, 2)));
//       // 由垂直分辨率获得
//       float distanceHorizonThreshold =fabs(
//         sin(angleVertical)) * sqrt(pow(_points_ring_list[i]->points[k].x, 2) + pow(_points_ring_list[i]->points[k].y, 2)) *
//         M_PI * _Angular_Res / 180;

//       float distancePre = abs(_points_ring_list[i]->points[k].z - _points_ring_list[i]->points[k - 1].z);
//       if (distancePre > distanceHorizonThreshold * _Distance_VerticalThres) {
//         _Distance_VerticlePointsIndex[i].push_back(k);
//       }
//     }


//   }


//   for (size_t i = 0; i < nScans; i++) {
//       _Index[i] = Vectors_Intersection(_Height_Points_Index[i], _Curvature_PointsIndex[i]);

//       if (_use_horizon) {
//         _Index[i] = Vectors_Intersection(_Index[i], _Distance_HorizonPointsIndex[i]);
//       }

//       if (_use_verticle) {
//         _Index[i] = Vectors_Intersection(_Index[i], _Distance_VerticlePointsIndex[i]);
//       }

//       // 最终得到的特征点索引存储在 _Index 中
//       extract_Points(_points_ring_list[i], _feature_points_list[i], boost::make_shared<std::vector<int>>(_Index[i]));

//       *feature_points += *_feature_points_list[i];
//   }
// }


}
