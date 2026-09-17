#pragma once

#ifndef LIDAR_BOUDART_DETECTION_H
#define LIDAR_BOUDART_DETECTION_H

#include "lidar_camera_road_curb_detection/core_head.hpp"
#include "lidar_camera_road_curb_detection/ground_seg.hpp"
#include "lidar_camera_road_curb_detection/feature.hpp"
#include "lidar_camera_road_curb_detection/cloud_id.hpp"
#include "lidar_camera_road_curb_detection/distinguish.hpp"
#include "lidar_camera_road_curb_detection/patchworkpp.hpp"
#include "lidar_camera_road_curb_detection/picture.hpp"
#include "lidar_camera_road_curb_detection/boundarypoints.hpp"
#include <thread>    // 对于std::thread
#include <mutex>     // 对于std::mutex
#include <condition_variable> // 对于std::condition_variable
namespace CurbDetection {
class ParamServer;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image> SyncPolicy;
class CurbDetector : public ParamServer{

public:
  CurbDetector();
  ~CurbDetector();

  bool init();

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs);

  void PointCloudCallback_with_camera(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs,const sensor_msgs::ImageConstPtr &image_msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr FilterCloud_condition(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

  pcl::PointCloud<PointXYZIRT>::Ptr FilterCloud_condition_XYZIRT(const pcl::PointCloud<PointXYZIRT>::Ptr& cloud,float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

  pcl::PointCloud<pcl::PointXYZI>::Ptr FilterCloud_cropbox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint);

  pcl::PointCloud<pcl::PointXYZI>::Ptr RANSACCloud_Line(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  void PointXYZIRT2PointXYZI(const pcl::PointCloud<PointXYZIRT> in_cloud_xyzirt,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);
  void PointXYZIRT2PointXYZI_2(const pcl::PointCloud<PointXYZIRT> in_cloud_xyzirt,pcl::PointCloud<pcl::PointXYZI> out_cloud);

  void PointXYZI2PointXYZIRT(const pcl::PointCloud<pcl::PointXYZI> in_cloud_xyzi,pcl::PointCloud<PointXYZIRT> out_cloud);

  std::deque<pcl::PointCloud<pcl::PointXYZI>> queue_XYZI;
  std::deque<pcl::PointCloud<PointXYZIRT>> queue_XYZIRT;


private:

  // 定义订阅话题的指针
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
  message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
  message_filters::Synchronizer<SyncPolicy> *sync; // 时间同步器
  cv::Mat image_in;
  float theta_r;
  float anlgus_;
  ros::NodeHandle nh_;
  ros::Publisher origin_pub_;
  ros::Publisher test_pub_11_;
  ros::Publisher test_pub_22_;
  ros::Publisher image_pub_;
  ros::Publisher Ground_point_;
  ros::Publisher NonGround_point_;

  ros::Publisher road_direction_;
  ros::Publisher feature_point_pub_;
  ros::Publisher all_left_curb_pub_;
  ros::Publisher all_right_curb_pub_;
  ros::Publisher left_curb_pub_;
  ros::Publisher right_curb_pub_;
  ros::Subscriber sub_cloud_;
  ros::Publisher left_curb_pcl1_pub_;
  ros::Publisher right_curb_pcl1_pub_;
  ros::Publisher left_curb_udp_pub_;
  ros::Publisher right_curb_udp_pub_;
  ros::Publisher pubMarker_;
  ros::Publisher pubMarker_pic_;

  ros::Publisher cloud_XYZI_pub_out;

  sensor_msgs::PointCloud2 ros_left_curb_;
  sensor_msgs::PointCloud2 ros_right_curb_;

  std::mutex mutex_;
  std::condition_variable cv_;
  bool image_processed_ = false;
  bool cloud_processed_ = false;
  // 用于存储线程处理结果的变量
//   struct ThreadResults {
//     cv::Mat image_out;
//     pcl::PointCloud<pcl::PointXYZI> cloud_road_direction;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints{new pcl::PointCloud<pcl::PointXYZI>};
//     pcl::PointCloud<pcl::PointXYZI> cloud_XYZI;
//     // pcl::PointCloud<pcl::PointXYZI> cloud_XYZI_;
//     // pcl::PointCloud<PointXYZIRT> cloud_XYZIRT;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr Ground_Points{new pcl::PointCloud<pcl::PointXYZI>};
//     pcl::PointCloud<pcl::PointXYZI>::Ptr NonGround_Points{new pcl::PointCloud<pcl::PointXYZI>};
//     int dir = 0;
//     double time_taken2 = 0.0;
// } thread_results_;
};

}

#endif // LIDAR_BOUDART_DETECTION_H
