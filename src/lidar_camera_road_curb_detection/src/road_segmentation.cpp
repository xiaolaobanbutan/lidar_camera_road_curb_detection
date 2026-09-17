#include <lidar_camera_road_curb_detection/road_segmentation.hpp>

namespace CurbDetection {
RoadSegmentation::RoadSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud)

{
  _completeCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  *_completeCloud = *incloud;
  _grid_map_vec.resize(360);
  _segmentAngle.resize(2);
  _distance_vec.resize(360);
  _distance_vec_filtered.resize(360);
  _distance_vec_.resize(360);
  _nearest_points.resize(360);

  road_central_line_.resize(2);
}

/**
 * @brief 把非地面点云按照一定分辨率划分不同组别,获得 _grid_map_vec 集合
 *
 */
void RoadSegmentation::generatePolarGrid() {
  // 这里处理经过滤除杂点后的非地面点云 obstacleCloudFiltered
  for (size_t i = 0; i < _completeCloud->points.size(); ++i) {

    float ori =
        std::atan2(_completeCloud->points[i].y, _completeCloud->points[i].x) *
        180 / M_PI;
    ori = ori < 0 ? ori + 360 : ori;
    // 与论文一致，光束带分辨率设置为 1 度
    float resolution = 1;
    int segment_index = (int)(ori / resolution);

    if (segment_index < 360) {
      // 二维投影矩阵，根据方向角划分
      _grid_map_vec[segment_index].push_back(_completeCloud->points[i]);
    }
  }
}

/**
 * @brief 计算 _grid_map_vec 每个组到激光雷达的距离
 *
 */
void RoadSegmentation::computeDistanceVec() {
#pragma omp parallel for schedule(runtime)
  for (size_t i = 0; i < _grid_map_vec.size(); ++i) {
    // 这里对 _grid_map_vec 每个组的点云数据按照到激光雷达的距离进行排序
    std::sort(_grid_map_vec[i].begin(), _grid_map_vec[i].end(),
              [](pcl::PointXYZI &left, pcl::PointXYZI &right) {
                float d1 = pow(left.x, 2) + pow(left.y, 2);
                float d2 = pow(right.x, 2) + pow(right.y, 2);
                return (d1 < d2);
              });

    if (_grid_map_vec[i].size() > 0) {

      // 计算 _grid_map_vec 每个组最小值点云和最大值点云
      double distance_min = pow(
          pow(_grid_map_vec[i][0].x, 2) + pow(_grid_map_vec[i][0].y, 2), 0.5);
      double distance_max =
          pow(pow(_grid_map_vec[i][_grid_map_vec[i].size() - 1].x, 2) +
                  pow(_grid_map_vec[i][_grid_map_vec[i].size() - 1].y, 2),
              0.5);

      // 获得距离最大点与最小点的比值 _distance_vec
      _distance_vec[i] = std::pair<float, int>(distance_min / distance_max, i);
      // _distance_vec_[i]=distance_min/distance_max;
      // pcl::PointXYZI point;
      // point.x = distance_min * cos(i * M_PI / 180);
      // point.y = distance_min * sin(i * M_PI / 180);
      // point.z = 0;
      // _nearest_points[i] = point;
    } else {
      // pcl::PointXYZI point;
      // point.x = 33 * cos(i * M_PI / 180);
      // point.y = 33 * sin(i * M_PI / 180);
      // point.z = 0;
      // 组内没有点的比值设置为1
      _distance_vec[i] = std::pair<float, int>(1.0, i);
      // _nearest_points[i] = point;
    }
  }
  //    vector<pair<double,int> > distance_vec_temp(360);
  //    distance_vec_temp[0]=_distance_vec[0];
  //    distance_vec_temp[1]=_distance_vec[1];
  //    distance_vec_temp[359]=_distance_vec[359];
  //    distance_vec_temp[358]=_distance_vec[358];

  // 获得前3组和后3组
  _distance_vec_front.push_back(_distance_vec[0]);
  _distance_vec_front.push_back(_distance_vec[1]);
  _distance_vec_front.push_back(_distance_vec[2]);
  _distance_vec_front.push_back(_distance_vec[359]);
  _distance_vec_front.push_back(_distance_vec[358]);
  _distance_vec_front.push_back(_distance_vec[357]);


  // 通过滑动窗口法来计算每个组与相邻组的中值
  for (size_t i = 3; i < _distance_vec.size() - 3; ++i) {
    std::vector<float> temp;

    // 对于每个比值的前后6个比值进行排序
    if (i >= 0 && i < 3) {
      //            vector<double> temp;
      temp.push_back(_distance_vec[i - 3 + 360].first);
      temp.push_back(_distance_vec[i - 2 + 360].first);
      temp.push_back(_distance_vec[i - 1 + 360].first);
      temp.push_back(_distance_vec[i].first);
      temp.push_back(_distance_vec[i + 1].first);
      temp.push_back(_distance_vec[i + 2].first);
      temp.push_back(_distance_vec[i + 3].first);
      std::sort(temp.begin(), temp.end());
    } else if (i >= 357 && i <= 359) {
      //            vector<double> temp;
      temp.push_back(_distance_vec[i - 3].first);
      temp.push_back(_distance_vec[i - 2].first);
      temp.push_back(_distance_vec[i - 1].first);
      temp.push_back(_distance_vec[i].first);
      temp.push_back(_distance_vec[i + 1 - 360].first);
      temp.push_back(_distance_vec[i + 2 - 360].first);
      temp.push_back(_distance_vec[i + 3 - 360].first);
      sort(temp.begin(), temp.end());
    } else {
      //            vector<double> temp;
      temp.push_back(_distance_vec[i - 3].first);
      temp.push_back(_distance_vec[i - 2].first);
      temp.push_back(_distance_vec[i - 1].first);
      temp.push_back(_distance_vec[i].first);
      temp.push_back(_distance_vec[i + 1].first);
      temp.push_back(_distance_vec[i + 2].first);
      temp.push_back(_distance_vec[i + 3].first);
      sort(temp.begin(), temp.end());
    }

    // 对于每个组的中值分配到左右
    if (i > 90 && i < 270) {
      _distance_vec_rear.push_back(std::pair<float, int>(temp[3], i));
    } else {
      _distance_vec_front.push_back(std::pair<float, int>(temp[3], i));
    }
    // _distance_vec_filtered[i] = temp[2];
  }
}

void RoadSegmentation::computeSegmentAngle() {
  // 对 _distance_vec_front 和 _distance_vec_rear 中的元素按距离进行降序排序。这样可以确保最大距离的点在数组的前面
  std::sort(_distance_vec_front.begin(), _distance_vec_front.end(),
            [](std::pair<float, int> &left, std::pair<float, int> &right) {
              return left.first > right.first;
            });

  std::sort(_distance_vec_rear.begin(), _distance_vec_rear.end(),
            [](std::pair<float, int> &left, std::pair<float, int> &right) {
              return left.first > right.first;
            });

  // 如果 _distance_vec_front 中第一个元素的距离为 1.0（即最大距离），则通过查找前方距离为 1.0 的点并计算它们的角度来确定分割角度。
  if (_distance_vec_front[0].first == 1.0) {
    std::vector<int> max_distance_angle_left;
    std::vector<int> max_distance_angle_right;

    for (size_t i = 0; i < _distance_vec_front.size(); ++i) {
      if (_distance_vec_front[i].first == 1.0 &&
          _distance_vec_front[i].second < 90) {
        max_distance_angle_left.push_back(_distance_vec_front[i].second);
      }
      if (_distance_vec_front[i].first == 1.0 &&
          _distance_vec_front[i].second > 270) {
        max_distance_angle_right.push_back(_distance_vec_front[i].second);
      }
    }

    std::sort(max_distance_angle_left.begin(), max_distance_angle_left.end(),
              [](int &left, int &right) { return left > right; });

    std::sort(max_distance_angle_right.begin(), max_distance_angle_right.end(),
              [](int &left, int &right) { return left > right; });

    // 将 max_distance_angle_right 向量中的元素插入到 max_distance_angle_left 向量
    max_distance_angle_left.insert(max_distance_angle_left.end(),
                                   max_distance_angle_right.begin(),
                                   max_distance_angle_right.end());

    if (!max_distance_angle_left.empty()) {
      _segmentAngle[0] =
          max_distance_angle_left[int(max_distance_angle_left.size() / 2)];
    } else {
      _segmentAngle[0] = 0;
    }

  } else {
    _segmentAngle[0] = _distance_vec_front[0].second;
  }

  // 计算后方segmentation angle
  if (_distance_vec_rear[0].first == 1.0) {
    std::vector<int> max_distance_angle;

    for (size_t i = 0; i < _distance_vec_rear.size(); ++i) {
      if (_distance_vec_rear[i].first == 1) {
        max_distance_angle.push_back(_distance_vec_rear[i].second);
      }
    }
    std::sort(max_distance_angle.begin(), max_distance_angle.end());
    if (!max_distance_angle.empty()) {
      _segmentAngle[1] = max_distance_angle[int(max_distance_angle.size() / 2)];
    } else {
      _segmentAngle[1] = 0;
    }

  } else {
    _segmentAngle[1] = _distance_vec_rear[0].second;
  }
  // cout << "The front segmentation angle is " << _segmentAngle[0] << endl;
  // cout << "The back segmentation angle is  " << _segmentAngle[1] << endl;

  for (size_t i = 0; i < road_central_line_.size(); ++i) {
    road_central_line_[i].type = visualization_msgs::Marker::LINE_LIST;
    road_central_line_[i].action = visualization_msgs::Marker::ADD;

    road_central_line_[i].id = i;

    road_central_line_[i].scale.x = 0.1;
    // Line list is green
    if (i == 0)
      road_central_line_[i].color.g = 1.0;
    else
      road_central_line_[i].color.r = 1.0;

    road_central_line_[i].color.a = 1.0;

    road_central_line_[i].pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(
            0, 0, _segmentAngle[i] / 180.0f * M_PI);

    geometry_msgs::Point p;
    p.x = p.y = p.z = 0;
    // 行列表每行需要两个点
    road_central_line_[i].points.push_back(p);

    p.x += 5.0;// 论文不是拥这个
    road_central_line_[i].points.push_back(p);
  }

}

void RoadSegmentation::process(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                               std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outcloud,
                               std::vector<visualization_msgs::Marker> &road_central_line) {

  // 以下三个函数用到的点云都是 obstacleCloudFiltered，非地面点

  // 把非地面点云按照 1 度的分辨率划分360个组别
  this->generatePolarGrid();
  // 计算每个组最大点云与最小点云之间的距离比值，组内没有点的比值设置为 1，再对于所有组使用中值滤波
  this->computeDistanceVec();
  // 选取前方比值为1的角度，所有角度加起来除以2就是道路方向，如果没有比值为1的，那么道路方向为0
  this->computeSegmentAngle();


  // 道路中心线
  road_central_line = road_central_line_;

  // 分割左右路沿候选点，粗分割，用的的点云是特征点 featurePoints
  // 在道路中心线左边即为左路沿候选点，右侧同理
  for (size_t i = 0; i < incloud->points.size(); ++i) {
    pcl::PointXYZI point(incloud->points[i]);
    //        point.intensity=i;
    // 选用在 x 轴上大于0的部分
    if (point.x > 0) {
      // x乘以角度判断左右
      if (point.y > point.x * tan(_segmentAngle[0] * M_PI / 180)) {
        outcloud[0]->points.push_back(point);
      } else {
        outcloud[1]->points.push_back(point);
      }
    // 选用在 x 轴上小于0的部分
    } else {
      if (point.y > point.x * tan(_segmentAngle[1] * M_PI / 180)) {
        outcloud[0]->points.push_back(point);
      } else {
        outcloud[1]->points.push_back(point);
      }
    }
  }

}


void RoadSegmentation::process_with_picture(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                               std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outcloud,
                               visualization_msgs::Marker &road_central_line,
                               int Dir_Angle
                               ) {

  road_direction_line_.type = visualization_msgs::Marker::LINE_LIST;
  road_direction_line_.action = visualization_msgs::Marker::ADD;
  road_direction_line_.id = 0;
  road_direction_line_.scale.x = 0.1;
  road_direction_line_.color.r = 1.0;
  road_direction_line_.color.g = 0.7;
  road_direction_line_.color.a = 1.0;
  road_direction_line_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, Dir_Angle / 180.0f * M_PI);

  geometry_msgs::Point p;
  p.x = p.y = p.z = 0;
  // 行列表每行需要两个点
  road_direction_line_.points.push_back(p);

  // p.x += 3.0;
  p.x += 10.0;
  road_direction_line_.points.push_back(p);

  // 道路中心线
  road_central_line = road_direction_line_;

  // cout<<"Dir_Angle:"<<Dir_Angle<<endl;

  // 分割左右路沿候选点，粗分割，用的的点云是特征点 featurePoints
  for (size_t i = 0; i < incloud->points.size(); ++i) {
    pcl::PointXYZI point(incloud->points[i]);
    // x乘以角度判断左右
    if (point.y > point.x * tan(Dir_Angle  * M_PI / 180))
    {
      outcloud[0]->points.push_back(point);
    } else {
      outcloud[1]->points.push_back(point);
    }
  }


}


} // namespace CurbDetection
