#pragma once

#ifndef BOUNDARYPOINTS_H
#define BOUNDARYPOINTS_H

#include "lidar_camera_road_curb_detection/core_head.hpp"
#include "lidar_camera_road_curb_detection/curb_detector.hpp"
#include "lidar_camera_road_curb_detection/ground_seg.hpp"
#include "lidar_camera_road_curb_detection/feature.hpp"
#include "lidar_camera_road_curb_detection/cloud_id.hpp"
#include "lidar_camera_road_curb_detection/distinguish.hpp"
#include "lidar_camera_road_curb_detection/road_segmentation.hpp"
#include "lidar_camera_road_curb_detection/grid_map.hpp"

namespace CurbDetection {

class BoundaryPoints{
public:
  BoundaryPoints(pcl::PointCloud<pcl::PointXYZI> &incloud, const cloud_msg &cmMsg,
                 const boundaryPointsMsg &bpMsg);

  void extractPointCloud(pcl::PointCloud<pcl::PointXYZI> &incloud, pcl::PointIndicesPtr indices,
                         pcl::PointCloud<pcl::PointXYZI> &outcloud);

  void lineFitRansac(pcl::PointCloud<pcl::PointXYZI> &incloud, pcl::PointIndices &indices);

  void statisticalFilter_indces(pcl::PointCloud<pcl::PointXYZI> incloud, int meanK,
                                pcl::PointIndicesPtr pointindices,
                                double stdThreshold);

  void pointcloud_projection(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                             pcl::PointCloud<pcl::PointXYZI> &outcloud);

  void ransac_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, float residualThreshold,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);

  void generateGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                    std::vector<pcl::PointCloud<pcl::PointXYZI>> &outcloud);

  void distanceFilterByGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud, bool left);

  void distanceFilterByLaserLeft(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);

  void distanceFilterByLaserRight(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);

  void process(pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud,
                pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_left,
                pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_right,
               std::vector<visualization_msgs::Marker> &road_central_line);


  void process_without_picture(pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud,
                pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_left,
                pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_right,
               std::vector<visualization_msgs::Marker> &road_central_line);

  void process_with_picture(
                pcl::PointCloud<pcl::PointXYZI>::Ptr ALL_Cloud_left,
                pcl::PointCloud<pcl::PointXYZI>::Ptr ALL_Cloud_right,
                pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_left,
                pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_right,
                pcl::PointCloud<pcl::PointXYZI>::Ptr initial_Cloud_left,
                pcl::PointCloud<pcl::PointXYZI>::Ptr intial_Cloud_right,
                pcl::PointCloud<pcl::PointXYZI>::Ptr left_filtered,
                pcl::PointCloud<pcl::PointXYZI>::Ptr right_filtered,
                visualization_msgs::Marker &road_central_line,
                int Dir_Angle,
                double &stime
                );


private:
  ros::NodeHandle nh1;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud;
  pcl::PointIndices _indicesLeft;
  pcl::PointIndices _indicesRight;

  ros::Publisher Leftcloud_initial_pub_out;
  ros::Publisher Rightcloud_initial_pub_out;
  // double _dbR;//密度直达点的搜索半径
  // double _neighborRate;
  // int _min_pets;//邻域内最少点个数

  float _rmseThres;
  float _meanThres;
  float _gridRes;
  int _gridNum;
  float _curveFitThres;
  bool _use_curve_fit;


  cloud_msg _cmMsg;
};
} // namespace CurbDetection

#endif // BOUNDARY_POINTS_H
