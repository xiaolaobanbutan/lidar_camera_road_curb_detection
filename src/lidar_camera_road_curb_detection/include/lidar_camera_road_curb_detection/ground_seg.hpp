#pragma once

#ifndef GROUND_SEG_H
#define GROUND_SEG_H

#include "lidar_camera_road_curb_detection/core_head.hpp"

namespace CurbDetection {

class Ground_Seg{
public:

  // 网格划分,构造函数

  Ground_Seg(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, ransacMsg ran);


  // 使用 RANSAC 方法拟合平面

  void RANSACCloud_Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         pcl::PointIndices::Ptr planeIndices,
                         pcl::ModelCoefficients::Ptr coefficients);

  // 索引点去除

  void extract_Ground(pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
                      pcl::PointIndices::Ptr Indices, bool setNeg = false);

  // 将上述模块一起处理

  void ground_proess(pcl::PointCloud<pcl::PointXYZI>::Ptr groundpoints,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr non_groundpoints
                    );


private:
  float _threshold;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _cloudptrlist;

};

} // namespace CurbDetection

#endif // GROUND_SEG_H
