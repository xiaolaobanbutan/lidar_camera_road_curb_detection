#pragma once

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "lidar_camera_road_curb_detection/core_head.hpp"

namespace CurbDetection {

class point2D {
public:
  float r;
  float z;

  point2D() {}

  point2D(pcl::PointXYZI point) {
    r = sqrt(pow(point.x, 2) + pow(point.y, 2));
    z = point.z;
  }
};

class GridMap {
public:
  GridMap(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, float gridRes, int gridNum);

  void generateCartesianGrid(std::vector<std::vector<pcl::PointXYZI>> &grid_map_vec_carte);

  void generateIDSCANGrid(std::vector<std::vector<pcl::PointXYZI>> &grid_map_vec_carte);

  void distanceFilterByCartesianGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud, bool left);

  void distanceFilterByCartesianGrid_with_ID_scan(pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud, bool left);

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr _origin_cloud_ptr;

  int _grid_num;
  float _grid_res;
};
}

#endif // GRID_MAP_H
