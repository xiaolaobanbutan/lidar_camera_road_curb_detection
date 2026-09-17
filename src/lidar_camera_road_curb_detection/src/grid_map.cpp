#include "lidar_camera_road_curb_detection/grid_map.hpp"

namespace CurbDetection {
/**
 * @brief Construct a new Grid Map:: Grid Map object
 *
 * @param incloud
 * @param gridRes 网格分辨率
 * @param gridNum 网格数量
 */
GridMap::GridMap(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, float gridRes, int gridNum)
    : _grid_res(gridRes), _grid_num(gridNum) {
  _origin_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
  *_origin_cloud_ptr = *incloud;
}

/**
 * @brief 将原始点云划分为笛卡尔网格，并将每个网格中的点存储在二维向量grid_map_vec_carte
 *
 * @param grid_map_vec_carte
 */
void GridMap::generateCartesianGrid(
    std::vector<std::vector<pcl::PointXYZI>> &grid_map_vec_carte) {

  // 初始化网格的大小， _grid_num 是网格的数量
  grid_map_vec_carte.resize(_grid_num);

  // 把点云按照 x 坐标分成条状，注意x轴应该是向前的
  for (int i = 0; i < _origin_cloud_ptr->points.size(); ++i) {


    // 当且仅当存在全范围点云使用 _grid_num / 2
    // int row = int(_origin_cloud_ptr->points[i].x / _grid_res + _grid_num / 2);

    int row = int(_origin_cloud_ptr->points[i].x / _grid_res);

    if (row >= 0 && row < grid_map_vec_carte.size()) {
      grid_map_vec_carte[row].push_back(_origin_cloud_ptr->points[i]);
    }
  }
}

void GridMap::generateIDSCANGrid(std::vector<std::vector<pcl::PointXYZI>> &grid_map_vec_carte) {

  grid_map_vec_carte.resize(64);

  // 把点云按照 x 坐标分成条状，注意x轴应该是向前的
  for (int i = 0; i < _origin_cloud_ptr->points.size(); ++i) {

    int scanidex = int(_origin_cloud_ptr->points[i].intensity);

    if (scanidex >= 0 && scanidex < grid_map_vec_carte.size()) {
      grid_map_vec_carte[scanidex].push_back(_origin_cloud_ptr->points[i]);
    }
  }
}

/**
 * @brief 在每个网格中找到具有最小y值的点，并将其添加到输出点云中
 *
 * @param outcloud
 * @param left 左侧点填 true，右侧点填 false
 */
void GridMap::distanceFilterByCartesianGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
                                            bool left) {

  // 设置点云网格容器
  std::vector<std::vector<pcl::PointXYZI>> gridLeft;

  // 放在函数中填充网格
  this->generateCartesianGrid(gridLeft);

  pcl::PointCloud<pcl::PointXYZI>::Ptr Leftcloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < gridLeft.size(); ++i) {
    if (gridLeft[i].size() > 0) {
      int min_y_Left = 0;

      for (size_t k = 0; k < gridLeft[i].size(); ++k) {

        if (left) {

          // 寻找该区域内 y 值最小的点的 ID，也就是最靠近 x 轴的点
          if (gridLeft[i][min_y_Left].y > gridLeft[i][k].y) {
            min_y_Left = k;
          }
        }

        else {
          // 在右侧的 y 坐标是负的, 所以是小于 ，一样寻找最靠近 x 轴的点
          if (gridLeft[i][min_y_Left].y < gridLeft[i][k].y) {
            min_y_Left = k;
          }
        }
      }

      // 获得左边或者右边最靠近 x 轴的点
      outcloud->points.push_back(gridLeft[i][min_y_Left]);
    }
  }
}

/**
 * @brief 在每个网格中找到具有最小y值的点，并将其添加到输出点云中
 *
 * @param outcloud
 * @param left 左侧点填 true，右侧点填 false
 */
void GridMap::distanceFilterByCartesianGrid_with_ID_scan(pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
                                            bool left) {

  // 设置点云网格容器
  std::vector<std::vector<pcl::PointXYZI>> gridLeft;

  // 放在函数中填充网格
  this->generateIDSCANGrid(gridLeft);

  pcl::PointCloud<pcl::PointXYZI>::Ptr Leftcloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < gridLeft.size(); ++i) {
    if (gridLeft[i].size() > 0) {
      int min_y_Left = 0;

      for (size_t k = 0; k < gridLeft[i].size(); ++k) {

        if (left) {

          // 寻找该区域内 y 值最小的点的 ID，也就是最靠近 x 轴的点
          if (gridLeft[i][min_y_Left].y > gridLeft[i][k].y) {
            min_y_Left = k;
          }
        }

        else {
          // 在右侧的 y 坐标是负的, 所以是小于 ，一样寻找最靠近 x 轴的点
          if (gridLeft[i][min_y_Left].y < gridLeft[i][k].y) {
            min_y_Left = k;
          }
        }
      }

      // 获得左边或者右边最靠近 x 轴的点
      outcloud->points.push_back(gridLeft[i][min_y_Left]);
    }
  }
}

}
