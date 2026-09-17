#ifndef ROAD_SEGMENTATION_H
#define ROAD_SEGMENTATION_H

#include "lidar_camera_road_curb_detection/core_head.hpp"

namespace CurbDetection {

class RoadSegmentation {
public:
  RoadSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud);
  void generatePolarGrid();
  void computeDistanceVec();
  void computeSegmentAngle();
  void process(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outcloud,
               std::vector<visualization_msgs::Marker> &road_central_line);

  void process_with_picture(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outcloud,
               visualization_msgs::Marker &road_central_line,int Dir_Angle);

  std::vector<pcl::PointXYZI> _nearest_points;
  std::vector<float> _distance_vec_filtered;
  std::vector<float> _distance_vec_;

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr _completeCloud;

  std::vector<std::vector<pcl::PointXYZI>> _grid_map_vec;

  std::vector<std::pair<float, int>> _distance_vec;
  std::vector<std::pair<float, int>> _distance_vec_front;
  std::vector<std::pair<float, int>> _distance_vec_rear;

  std::vector<int> _segmentAngle;

  std::vector<visualization_msgs::Marker> road_central_line_;

  visualization_msgs::Marker road_direction_line_;

};

} // namespace CurbDetection

#endif // ROAD_SEGMENTATION_H
