#pragma once

#ifndef CLOUD_ID_H
#define CLOUD_ID_H

#include "lidar_camera_road_curb_detection/core_head.hpp"



namespace CurbDetection {
class Cloud_id {
public:
  Cloud_id(const Cloud_Msg &cmMsg);

  ~Cloud_id();

  const float &getLowerBound() { return _lowerBound; } // 雷达的最小检测角度
  const float &getUpperBound() { return _upperBound; } // 雷达的最大检测角度
  const int &getNumberOfScanRings() { return _nScanRings; } // 雷达线数：32线或者64线

  int getRingForAngle(const float &angle);

  // 激光雷达的点云按照时间顺序排序，其中kitti的雷达扫描线从上到下扫描，只要区分扫描线即可

  void processByOri(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);

  void processByOri_XYZI2XYZIRT(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                    pcl::PointCloud<PointXYZIRT>::Ptr outcloud);

  void processByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
                          scanIndices &scanindices
                          );

  void processByRing(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                          std::vector<std::vector<size_t>> &points_by_ring
                          );



  // 据点的垂直角度信息将点云数据分组，并在输出点云中记录激光线编号，但是这个方法精度不高

  // void ProcessByAngle(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
  //                     pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);

  // 据点的垂直角度信息将点云数据分组，并在输出点云中记录激光线编号，但是这个方法精度不高

  // void processByVer(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
  //                   pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
  //                   scanIndices &scanindices);



  float _lowerBound; // 雷达的最小检测角度
  float _upperBound; // 雷达的最大检测角度
  int _nScanRings;   // 雷达线数：32线或者64线
  float _interval;     // 每条雷达扫描线之间的间隔是 1/_factor ，单位度
};

} // namespace CurbDetection

#endif
