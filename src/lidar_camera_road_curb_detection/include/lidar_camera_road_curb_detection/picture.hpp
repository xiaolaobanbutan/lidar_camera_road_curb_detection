#pragma once

#ifndef PICTURE_H
#define PICTURE_H

#include "lidar_camera_road_curb_detection/core_head.hpp"
namespace CurbDetection {

class picture_proess {

public:
  picture_proess(const cv::Mat image,pcl::PointCloud<pcl::PointXYZI> &incloud,Picture_Msg picMsg);

  // 大津二值化算法
  void picture_cut(cv::Mat image_orgin,cv::Mat& image_cut);

  float arctan_y1x(pcl::PointXYZI pxy);

  void picture_out(cv::Mat& image_out_, double &time_take,pcl::PointCloud<pcl::PointXYZI>& road_direction,int& dir);
private:
  cv::Mat image_in_;
  cv::Mat image_t_;
  int gaussianblur_H_;
  int gaussianblur_W_;
  int canny_up_;
  int canny_low_;
  int threshold_thresh_;
  int threshold_maxVal_;
  std::string weight_path_;
  bool use_cuda_ = false;
  std::vector<std::vector<cv::Point>> chedaoxian;
  pcl::PointCloud<pcl::PointXYZI> incloud_;



  Eigen::MatrixXd C_T_L_; // 3x4的转换矩阵
  Eigen::Vector4d pointCloud_L_; // 点云坐标(x,y,z,1)
  Eigen::Vector3d pointCloud_C_; // 经过外参矩阵旋转平移变换后的坐标(u,v,w)

};
}
#endif
