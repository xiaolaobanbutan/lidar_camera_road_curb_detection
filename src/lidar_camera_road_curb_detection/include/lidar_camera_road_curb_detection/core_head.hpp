#pragma once


#ifndef CORE_HEAD_H
#define CORE_HEAD_H

#define PCL_NO_PRECOMPILE

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <vector>
#include <ctime>
#include <time.h>
#include <chrono>
#include <cmath>
#include <array>
#include <math.h>
#include <signal.h>
#include "log.h"
#include "math.h"
#include <sstream>
#include <vector>
#include <map>

// #include "opencv2/opencv.hpp"
#include <tf/tf.h>



#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <fstream>

// PCL for transformation
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>

/*ramer-douglas-peucker*/
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign.hpp>

// #include <lidar_msgs/Curb.h>


// CLASSES
#define PI acos(-1)
#define SENSOR_HEIGHT 1.73
#define UNLABELED 0
#define OUTLIER 1
#define NUM_ALL_CLASSES 34
#define ROAD 40
#define PARKING 44
#define SIDEWALKR 48
#define OTHER_GROUND 49
#define BUILDING 50
#define FENSE 51
#define LANE_MARKING 60
#define VEGETATION 70
#define TERRAIN 72
#define TRUEPOSITIVE 3
#define TRUENEGATIVE 2
#define FALSEPOSITIVE 1
#define FALSENEGATIVE 0
#define MARKER_Z_VALUE -2.2
#define UPRIGHT_ENOUGH 0.55
#define FLAT_ENOUGH 0.2
#define TOO_HIGH_ELEVATION 0.0
#define TOO_TILTED 1.0
#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;



// std::pair主要的作用是将两个数据组合成一个数据
typedef std::pair<size_t, size_t> IndexRange; // 一个扫描圈ID的范围

typedef std::vector<IndexRange> scanIndices; // 包含所有扫描圈ID的索引,假设每一圈的点都没有过滤掉，每一圈 N 个点，则scanindices[0]=(0,N-1),scanindices[1]=(N-1,2N-2),scanindices[2]=(2N-2,3N-3)...

typedef struct Point_3D {
    float x, y, z;
}point_3d ;




typedef struct cloud_msg {
  float lowerBound; // 雷达的最小检测角度
  float upperBound; // 雷达的最大检测角度
  int nScanRings;   // 雷达线数：32线或者64线
} Cloud_Msg;   //创建别名

typedef struct picture_msg {
  int canny_low;
  int canny_up;
  int GaussianBlur_width;
  int GaussianBlur_heigt;

  int threshold_thresh;
  int threshold_maxVal;

  std::string weight_path;
  bool use_cuda = false;


  std::vector<float> C_T_L;
} Picture_Msg;   //创建别名


typedef struct boundarypointsmsg {
  float rmseThres;
  float meanThres;
  int gridNum;
  float gridRes;
  float curveFitThres;
  bool useCurveRansac;
} boundaryPointsMsg;


typedef struct PCURVATURE{
	int index;
  float pc1;
  float pc2;
	float curvature;
}PCURVATURE;


typedef struct RANSACMsg {
  float segThres;
} ransacMsg;

// 自定义点云
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    uint16_t label;                     ///< 点标签
    // uint16_t id;
    // float timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // 强制SSE填充以实现正确的内存对齐
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
(float,x,x)
(float,y,y)
(float,z,z)
(float, intensity, intensity)
(uint16_t,ring,ring)
// (float ,timestamp,timestamp)
// (std::uint16_t, id, id)
)


struct PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< 激光强度读数
  uint16_t label;                     ///< 点标签
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // 确保正确对齐
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(std::uint16_t, label, label)
(std::uint16_t, id, id)
)





// 特征点信息
typedef struct featurepointsmsg {
  int heightRegion;
  float heightSigmaThre;
  float heightMaxThres;
  float heightMinThres;
  float curvatureThres;
  int curvatureRegion;
  float distanceHorizonThres;
  float distanceVerticalThres;
  float angularRes;
  bool useVerticle;
  bool useHorizon;
  float allCurvates_H;
  int nScanRings;

  float distinguish_angular;

  bool useAngleHorizon;
  int testRegion;
  float testThres;

} featurePointsMsg;


typedef struct judgemsg{
int MaxSacnID;
int MinSacnID;
float MinAnglus;
float Max_Hight_Cost;
float Max_Angle_Cost;
float Max_XY_Cost;
float angularRes;
// int ID_test;
float Ring_diff;
float Vertical_Res;
int nScanRings;
} JUDGEMSG;

typedef struct patchwork_struct{
  int num_iter;
  int num_lpr;
  int num_min_pts;
  int num_zones;
  int num_rings_of_interest;

  std::vector<int> num_sectors_each_zone;
  std::vector<int> num_rings_each_zone;
  std::vector<double> sector_sizes;
  std::vector<double> ring_sizes;
  std::vector<double> min_ranges;
  std::vector<double> elevation_thr;
  std::vector<double> flatness_thr;

  double sensor_height;
  double th_seeds;
  double th_dist;
  double th_seeds_v;
  double th_dist_v;
  double max_range;
  double min_range;
  double uprightness_thr;
  double adaptive_seed_selection_margin;
  double min_range_z2; // 12.3625
  double min_range_z3; // 22.025
  double min_range_z4; // 41.35
  double RNR_ver_angle_thr;
  double RNR_intensity_thr;

  bool verbose;
  bool enable_RNR;
  bool enable_RVPF;
  bool enable_TGR;
  int max_flatness_storage;
  int max_elevation_storage;

  bool visualize;
  bool groud_visualize;
};






namespace  CurbDetection{
class ParamServer {
  public:
    ros::NodeHandle nh_;
    float min_x,max_x,min_y,max_y,min_z,max_z;
    bool Is_XYZIRT;
    bool without_camera;
    bool allow_groud_visualize;
    bool use_pacthwork_;
    int test_; // 测试专用
    std::string cloud_topic;
    std::string camera_topic;

    featurePointsMsg Points_Msg; // 外部定义的参数
    Cloud_Msg laser_msg;
    judgemsg Distinguish_Msg;
    patchwork_struct patchwork_value;
    boundaryPointsMsg bpMsg;
    Picture_Msg picMsg;
    ransacMsg ransacc;


    ParamServer(){

      // Read parameters using nh_.getParam
      nh_.getParam("Is_XYZIRT", Is_XYZIRT);
      nh_.getParam("without_camera", without_camera);
      nh_.getParam("use_pacthwork",use_pacthwork_);
      nh_.getParam("test",test_);

      nh_.getParam("lowerBound", laser_msg.lowerBound);
      nh_.getParam("upperBound", laser_msg.upperBound);
      nh_.getParam("nScanRings", laser_msg.nScanRings);
      nh_.getParam("cloud_topic", cloud_topic);
      nh_.getParam("camera_topic", camera_topic);
      // Points_Msg.search_number;
      nh_.getParam("heightMaxThres", Points_Msg.heightMaxThres);
      nh_.getParam("heightMinThres", Points_Msg.heightMinThres);
      nh_.getParam("heightRegion", Points_Msg.heightRegion);
      nh_.getParam("heightSigmaThre", Points_Msg.heightSigmaThre);
      nh_.getParam("curvatureRegion", Points_Msg.curvatureRegion);
      nh_.getParam("curvatureThres", Points_Msg.curvatureThres);
      nh_.getParam("distanceHorizonThres", Points_Msg.distanceHorizonThres);
      nh_.getParam("distanceVerticalThres", Points_Msg.distanceVerticalThres);
      nh_.getParam("angularRes", Points_Msg.angularRes);
      nh_.getParam("useVerticle", Points_Msg.useVerticle);
      nh_.getParam("useHorizon", Points_Msg.useHorizon);
      nh_.getParam("allCurvates_H", Points_Msg.allCurvates_H);
      nh_.getParam("nScanRings", Points_Msg.nScanRings);
      nh_.getParam("testRegion", Points_Msg.testRegion);
      nh_.getParam("testThres", Points_Msg.testThres);
      nh_.getParam("useAngleHorizon", Points_Msg.useAngleHorizon);

      // Declare and read other parameters
      nh_.getParam("MaxSacnID", Distinguish_Msg.MaxSacnID);
      nh_.getParam("MinSacnID", Distinguish_Msg.MinSacnID);
      nh_.getParam("MinAnglus", Distinguish_Msg.MinAnglus);
      nh_.getParam("Max_Hight_Cost", Distinguish_Msg.Max_Hight_Cost);
      nh_.getParam("Max_Angle_Cost", Distinguish_Msg.Max_Angle_Cost);
      nh_.getParam("Max_XY_Cost", Distinguish_Msg.Max_XY_Cost);
      nh_.getParam("Ring_diff", Distinguish_Msg.Ring_diff);
      nh_.getParam("Vertical_Res", Distinguish_Msg.Vertical_Res);
      nh_.getParam("nScanRings", Distinguish_Msg.nScanRings);
      // nh_.getParam("ID_test", Distinguish_Msg.ID_test);
      // nh_.getParam("road_min_x", min_x);
      // nh_.getParam("road_max_x", max_x);
      // nh_.getParam("road_min_y", min_y);
      // nh_.getParam("road_max_y", max_y);
      // nh_.getParam("road_min_z", min_z);
      // nh_.getParam("road_max_z", max_z);

      nh_.getParam("rmseThres", bpMsg.rmseThres);
      nh_.getParam("meanThres", bpMsg.meanThres);
      nh_.getParam("gridNum", bpMsg.gridNum);
      nh_.getParam("gridRes", bpMsg.gridRes);
      nh_.getParam("curveFitThres", bpMsg.curveFitThres);
      nh_.getParam("useCurveRansac", bpMsg.useCurveRansac);
      nh_.getParam("segThres", ransacc.segThres);













      nh_.param("verbose", patchwork_value.verbose, false);
      nh_.param("sensor_height", patchwork_value.sensor_height, 1.723);
      nh_.param("num_iter", patchwork_value.num_iter, 3);
      nh_.param("num_lpr", patchwork_value.num_lpr, 20);
      nh_.param("num_min_pts", patchwork_value.num_min_pts, 10);
      nh_.param("th_seeds", patchwork_value.th_seeds, 0.4);
      nh_.param("th_dist", patchwork_value.th_dist, 0.3);
      nh_.param("th_seeds_v", patchwork_value.th_seeds_v, 0.4);
      nh_.param("th_dist_v", patchwork_value.th_dist_v, 0.3);
      nh_.param("max_range", patchwork_value.max_range, 80.0);
      nh_.param("min_range", patchwork_value.min_range, 2.7);
      nh_.param("uprightness_thr", patchwork_value.uprightness_thr, 0.5);
      nh_.param("adaptive_seed_selection_margin", patchwork_value.adaptive_seed_selection_margin, -1.1);
      nh_.param("RNR_ver_angle_thr", patchwork_value.RNR_ver_angle_thr, -15.0);
      nh_.param("RNR_intensity_thr", patchwork_value.RNR_intensity_thr, 0.2);
      nh_.param("max_flatness_storage", patchwork_value.max_flatness_storage, 1000);
      nh_.param("max_elevation_storage", patchwork_value.max_elevation_storage, 1000);
      nh_.param("enable_RNR", patchwork_value.enable_RNR, true);
      nh_.param("enable_RVPF", patchwork_value.enable_RVPF, true);
      nh_.param("enable_TGR", patchwork_value.enable_TGR, true);
      nh_.param("visualize", patchwork_value.visualize, false);
      nh_.param("groud_visualize", allow_groud_visualize, false);

      nh_.getParam("czm/num_zones", patchwork_value.num_zones);
      nh_.getParam("czm/num_sectors_each_zone", patchwork_value.num_sectors_each_zone);
      nh_.getParam("czm/mum_rings_each_zone", patchwork_value.num_rings_each_zone);
      nh_.getParam("czm/elevation_thresholds", patchwork_value.elevation_thr);
      nh_.getParam("czm/flatness_thresholds", patchwork_value.flatness_thr);

      nh_.getParam("canny_low", picMsg.canny_low);
      nh_.getParam("canny_up", picMsg.canny_up);
      nh_.getParam("GaussianBlur_heigt", picMsg.GaussianBlur_heigt);
      nh_.getParam("GaussianBlur_width", picMsg.GaussianBlur_width);
      nh_.getParam("threshold_thresh", picMsg.threshold_thresh);
      nh_.getParam("threshold_maxVal", picMsg.threshold_maxVal);
      nh_.getParam("weight_path", picMsg.weight_path);
      nh_.param("use_cuda", picMsg.use_cuda, false);
      nh_.getParam("C_T_L", picMsg.C_T_L);

    usleep(100);
  }

  ~ParamServer() {}
};




class PP{

public:


int NUM_ZEROS = 5;
double VEGETATION_THR = - SENSOR_HEIGHT * 3 / 4;

void PointXYZILID2XYZI(pcl::PointCloud<PointXYZILID>& src,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr dst){
  dst->points.clear();
  for (const auto &pt: src.points){
    pcl::PointXYZI pt_xyzi;
    pt_xyzi.x = pt.x;
    pt_xyzi.y = pt.y;
    pt_xyzi.z = pt.z;
    pt_xyzi.intensity = pt.intensity;
    dst->points.push_back(pt_xyzi);
  }
}

std::vector<int> outlier_classes = {UNLABELED, OUTLIER};
std::vector<int> ground_classes = {ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING, VEGETATION, TERRAIN};
std::vector<int> ground_classes_except_terrain = {ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING};
std::vector<int> traversable_ground_classes = {ROAD, PARKING, LANE_MARKING, OTHER_GROUND};

int count_num_ground(const pcl::PointCloud<PointXYZILID>& pc){
  int num_ground = 0;

  std::vector<int>::iterator iter;

  for (auto const& pt: pc.points){
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()){ // corresponding class is in ground classes
      if (pt.label == VEGETATION){
        if (pt.z < VEGETATION_THR){
           num_ground ++;
        }
      }else num_ground ++;
    }
  }
  return num_ground;
}

int count_num_ground_without_vegetation(const pcl::PointCloud<PointXYZILID>& pc){
  int num_ground = 0;

  std::vector<int>::iterator iter;

  std::vector<int> classes = {ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING, TERRAIN};

  for (auto const& pt: pc.points){
    iter = std::find(classes.begin(), classes.end(), pt.label);
    if (iter != classes.end()){ // corresponding class is in ground classes
      num_ground ++;
    }
  }
  return num_ground;
}

std::map<int, int> set_initial_gt_counts(std::vector<int>& gt_classes){
  std::map<int, int> gt_counts;
  for (int i = 0; i< gt_classes.size(); ++i){
    gt_counts.insert(std::pair<int,int>(gt_classes.at(i), 0));
  }
  return gt_counts;
}

std::map<int, int> count_num_each_class(const pcl::PointCloud<PointXYZILID>& pc){
  int num_ground = 0;
  auto gt_counts = set_initial_gt_counts(ground_classes);
  std::vector<int>::iterator iter;

  for (auto const& pt: pc.points){
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()){ // corresponding class is in ground classes
      if (pt.label == VEGETATION){
        if (pt.z < VEGETATION_THR){
           gt_counts.find(pt.label)->second++;
        }
      }else gt_counts.find(pt.label)->second++;
    }
  }
  return gt_counts;
}

int count_num_outliers(const pcl::PointCloud<PointXYZILID>& pc){
  int num_outliers = 0;

  std::vector<int>::iterator iter;

  for (auto const& pt: pc.points){
    iter = std::find(outlier_classes.begin(), outlier_classes.end(), pt.label);
    if (iter != outlier_classes.end()){ // corresponding class is in ground classes
      num_outliers ++;
    }
  }
  return num_outliers;
}


void discern_ground(const pcl::PointCloud<PointXYZILID>& src, pcl::PointCloud<PointXYZILID>& ground, pcl::PointCloud<PointXYZILID>& non_ground){
  ground.clear();
  non_ground.clear();
  std::vector<int>::iterator iter;
  for (auto const& pt: src.points){
    if (pt.label == UNLABELED || pt.label == OUTLIER) continue;
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()){ // corresponding class is in ground classes
      if (pt.label == VEGETATION){
        if (pt.z < VEGETATION_THR){
          ground.push_back(pt);
        }else non_ground.push_back(pt);
      }else  ground.push_back(pt);
    }else{
      non_ground.push_back(pt);
    }
  }
}

void discern_ground_without_vegetation(const pcl::PointCloud<PointXYZILID>& src, pcl::PointCloud<PointXYZILID>& ground, pcl::PointCloud<PointXYZILID>& non_ground){
  ground.clear();
  non_ground.clear();
  std::vector<int>::iterator iter;
  for (auto const& pt: src.points){
    if (pt.label == UNLABELED || pt.label == OUTLIER) continue;
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()){ // corresponding class is in ground classes
      if (pt.label != VEGETATION) ground.push_back(pt);
    }else{
      non_ground.push_back(pt);
    }
  }
}


void calculate_precision_recall(const pcl::PointCloud<PointXYZILID>& pc_curr,
                                pcl::PointCloud<PointXYZILID>& ground_estimated,
                                double & precision,
                                double& recall,
                                bool consider_outliers=true){

  int num_ground_est = ground_estimated.points.size();
  int num_ground_gt = count_num_ground(pc_curr);
  int num_TP = count_num_ground(ground_estimated);
  if (consider_outliers){
    int num_outliers_est = count_num_outliers(ground_estimated);
    precision = (double)(num_TP)/(num_ground_est - num_outliers_est) * 100;
    recall = (double)(num_TP)/num_ground_gt * 100;
  }else{
    precision = (double)(num_TP)/num_ground_est * 100;
    recall = (double)(num_TP)/num_ground_gt * 100;
  }
}

void calculate_precision_recall_without_vegetation(const pcl::PointCloud<PointXYZILID>& pc_curr,
                                                   pcl::PointCloud<PointXYZILID>& ground_estimated,
                                                   double & precision,
                                                   double& recall,
                                                   bool consider_outliers=true){
  int num_veg = 0;
  for (auto const& pt: ground_estimated.points)
  {
    if (pt.label == VEGETATION) num_veg++;
  }

  int num_ground_est = ground_estimated.size() - num_veg;
  int num_ground_gt = count_num_ground_without_vegetation(pc_curr);
  int num_TP = count_num_ground_without_vegetation(ground_estimated);
  if (consider_outliers){
    int num_outliers_est = count_num_outliers(ground_estimated);
    precision = (double)(num_TP)/(num_ground_est - num_outliers_est) * 100;
    recall = (double)(num_TP)/num_ground_gt * 100;
  }else{
    precision = (double)(num_TP)/num_ground_est * 100;
    recall = (double)(num_TP)/num_ground_gt * 100;
  }
}


void save_all_labels(const pcl::PointCloud<PointXYZILID>& pc, std::string ABS_DIR, std::string seq, int count){

  std::string count_str = std::to_string(count);
  std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
  std::string output_filename = ABS_DIR + "/" + seq + "/" + count_str_padded + ".csv";
  ofstream sc_output(output_filename);

  std::vector<int> labels(NUM_ALL_CLASSES, 0);
  for (auto const& pt: pc.points){
    if (pt.label == 0) labels[0]++;
    else if (pt.label == 1) labels[1]++;
    else if (pt.label == 10) labels[2]++;
    else if (pt.label == 11) labels[3]++;
    else if (pt.label == 13) labels[4]++;
    else if (pt.label == 15) labels[5]++;
    else if (pt.label == 16) labels[6]++;
    else if (pt.label == 18) labels[7]++;
    else if (pt.label == 20) labels[8]++;
    else if (pt.label == 30) labels[9]++;
    else if (pt.label == 31) labels[10]++;
    else if (pt.label == 32) labels[11]++;
    else if (pt.label == 40) labels[12]++;
    else if (pt.label == 44) labels[13]++;
    else if (pt.label == 48) labels[14]++;
    else if (pt.label == 49) labels[15]++;
    else if (pt.label == 50) labels[16]++;
    else if (pt.label == 51) labels[17]++;
    else if (pt.label == 52) labels[18]++;
    else if (pt.label == 60) labels[19]++;
    else if (pt.label == 70) labels[20]++;
    else if (pt.label == 71) labels[21]++;
    else if (pt.label == 72) labels[22]++;
    else if (pt.label == 80) labels[23]++;
    else if (pt.label == 81) labels[24]++;
    else if (pt.label == 99) labels[25]++;
    else if (pt.label == 252) labels[26]++;
    else if (pt.label == 253) labels[27]++;
    else if (pt.label == 254) labels[28]++;
    else if (pt.label == 255) labels[29]++;
    else if (pt.label == 256) labels[30]++;
    else if (pt.label == 257) labels[31]++;
    else if (pt.label == 258) labels[32]++;
    else if (pt.label == 259) labels[33]++;
  }

  for (uint8_t i=0; i < NUM_ALL_CLASSES;++i){
    if (i!=33){
      sc_output<<labels[i]<<",";
    }else{
      sc_output<<labels[i]<<endl;
    }
  }
  sc_output.close();
}

void save_all_accuracy(const pcl::PointCloud<PointXYZILID>& pc_curr,
                      pcl::PointCloud<PointXYZILID>& ground_estimated, std::string acc_filename,
                      double& accuracy, std::map<int, int>&pc_curr_gt_counts, std::map<int, int>&g_est_gt_counts){


//  std::cout<<"debug: "<<acc_filename<<std::endl;
  ofstream sc_output2(acc_filename, ios::app);

  int num_True = count_num_ground(pc_curr);
  int num_outliers_gt = count_num_outliers(pc_curr);
  int num_outliers_est = count_num_outliers(ground_estimated);

  int num_total_est = ground_estimated.points.size() - num_outliers_est;
  int num_total_gt = pc_curr.points.size() - num_outliers_gt;

  int num_False = num_total_gt - num_True;
  int num_TP = count_num_ground(ground_estimated);
  int num_FP = num_total_est - num_TP;
  accuracy = static_cast<double>(num_TP + (num_False - num_FP)) / num_total_gt * 100.0;

  pc_curr_gt_counts = count_num_each_class(pc_curr);
  g_est_gt_counts = count_num_each_class(ground_estimated);

  // save output
  for (auto const& class_id: ground_classes){
    sc_output2<<g_est_gt_counts.find(class_id)->second<<","<<pc_curr_gt_counts.find(class_id)->second<<",";
  }
  sc_output2<<accuracy<<endl;

  sc_output2.close();
}

void pc2pcdfile(const pcl::PointCloud<PointXYZILID>& TP, const pcl::PointCloud<PointXYZILID>& FP,
                const pcl::PointCloud<PointXYZILID>& FN, const pcl::PointCloud<PointXYZILID>& TN,
                std::string pcd_filename){
  pcl::PointCloud<pcl::PointXYZI> pc_out;

  for (auto const pt: TP.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = TRUEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt: FP.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = FALSEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt: FN.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = FALSENEGATIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt: TN.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = TRUENEGATIVE;
    pc_out.points.push_back(pt_est);
  }
  pc_out.width = pc_out.points.size();
  pc_out.height = 1;
  pcl::io::savePCDFileASCII(pcd_filename, pc_out);

}

};


}



#endif
