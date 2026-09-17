// 用于给点云命名id，最外圈为第一圈

#include "lidar_camera_road_curb_detection/cloud_id.hpp"

namespace CurbDetection {


Cloud_id::Cloud_id(const Cloud_Msg &cmMsg) {
  _lowerBound = cmMsg.lowerBound; // 雷达的最小检测角度
  _upperBound = cmMsg.upperBound; // 雷达的最大检测角度
  _nScanRings = cmMsg.nScanRings; // 雷达线数：32线或者64线

  // 每条雷达扫描线之间的间隔是 _interval ，单位度
  _interval = (_upperBound - _lowerBound) / (_nScanRings - 1) ;
}

Cloud_id::~Cloud_id(){}


// 计算垂直角度对应的扫描环编号，+0.5是为了有一定容错
int Cloud_id::getRingForAngle(const float &angle) {
  // 将角度转换为弧度，然后乘以180/pi，将其转换为度数
  return int(( (angle * 180 / M_PI) - _lowerBound) / _interval + 0.5);
}


/**
 * @brief 据点的垂直角度信息将点云数据分组，并在输出点云中记录激光线编号。
 *
 * @param incloud 原始点云
 * @param outcloud 非空点云集合
 * @param scanindices
//  */
// void Cloud_id::processByVer(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
//                             pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
//                             scanIndices &scanindices) {

//   size_t cloudSize = incloud->points.size(); // 点云数目

//   std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(_nScanRings); // 某一圈的所有点云


//   for (int i = 1; i < cloudSize; i++) {
//     // 排除空点
//     if (!pcl_isfinite(incloud->points[i].x) ||
//         !pcl_isfinite(incloud->points[i].y) ||
//         !pcl_isfinite(incloud->points[i].z)) {
//       continue;
//     }

//     // 排除零点
//     if (incloud->points[i].x * incloud->points[i].x
//         + incloud->points[i].y * incloud->points[i].y
//         + incloud->points[i].z * incloud->points[i].z <0.0001)
//         {continue;}

//     // 计算垂直点角度和雷达线的ID
//     float angle =std::atan(incloud->points[i].z /
//                  std::sqrt(pow(incloud->points[i].x,2)  + pow(incloud->points[i].y, 2)));

//     int scanID = getRingForAngle(angle);

//     if (scanID >= getNumberOfScanRings() || scanID < 0) {
//       continue;
//     }

//     laserCloudScans[scanID].push_back(incloud->points[i]);
//   }

//   // 输出
//   cloudSize = 0;
//   for (int i = 0; i < getNumberOfScanRings(); i++) {
//     if (laserCloudScans[i].size() > 0) {
//       (*outcloud) += laserCloudScans[i];
//       IndexRange range(cloudSize, 0);
//       cloudSize += laserCloudScans[i].size();
//       range.second = cloudSize > 0 ? cloudSize - 1 : 0;
//       scanindices.push_back(range);
//     }
//   }
// }










// /**
//  * @brief 据点的垂直角度信息将点云数据分组，并在输出点云中记录激光线编号。
//  *
//  * @param incloud 原始点云
//  * @param outcloud 非空点云集合
//  */
// void Cloud_id::ProcessByAngle(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
//                               pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud) {

//   size_t cloudSize = incloud->points.size(); // 点云数目

//   std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(_nScanRings); // 某一圈的所有点云


//   for (int i = 1; i < cloudSize; i++) {
//     // 排除空点
//     if (!pcl_isfinite(incloud->points[i].x) ||
//         !pcl_isfinite(incloud->points[i].y) ||
//         !pcl_isfinite(incloud->points[i].z)) {
//       continue;
//     }

//     // 排除零点
//     if (incloud->points[i].x * incloud->points[i].x
//         + incloud->points[i].y * incloud->points[i].y
//         + incloud->points[i].z * incloud->points[i].z <0.0001)
//         {continue;}

//     // 计算垂直点角度和雷达线的ID
//     float angle =std::atan(incloud->points[i].z /
//                  std::sqrt(pow(incloud->points[i].x,2)  + pow(incloud->points[i].y, 2)));

//     int scanID = getRingForAngle(angle);

//     if (scanID >= getNumberOfScanRings() || scanID < 0) {
//       continue;
//     }

//     laserCloudScans[scanID].push_back(incloud->points[i]);
//   }


//   for (int i = 0; i < getNumberOfScanRings(); i++) {
//     if (laserCloudScans[i].size() > 0) {
//       for (int j = 0;j<laserCloudScans[i].size();j++){
//         laserCloudScans[i].points[j].intensity = i;
//       }
//       (*outcloud) += laserCloudScans[i];
//     }
//   }
// }


/**
 * @brief  建立扫描线的索引范围scanindices, 假设每一圈的点都没有过滤掉，每一圈 N 个点，则scanindices[0]=(0,N-1),scanindices[1]=(N-1,2N-2),scanindices[2]=(2N-2,3N-3)...

 *
 * @param incloud
 * @param outcloud
 * @param scanindices
 */
void Cloud_id::processByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
                                  scanIndices &scanindices
                                  )
{
  // 获取输入点云的大小
  size_t cloudSize = incloud->points.size();

  std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(_nScanRings);

  int scanID = 0;



  // 从输入点云中根据 intensity 把不同的扫描线上的点云分类保存
  for (int i = 0; i < cloudSize; i++) {
    scanID = incloud->points[i].intensity;

    // 原始分类点云
    laserCloudScans[scanID].push_back(incloud->points[i]);

  }


  // 建立扫描线的索引范围range
  // 假设每一圈的点都没有过滤掉，每一圈 N 个点，则scanindices[0]=(0,N-1),scanindices[1]=(N-1,2N-2),scanindices[2]=(2N-2,3N-3)...
  cloudSize = 0;

  for (int i = 0; i < getNumberOfScanRings(); i++) {

    // 去除空的扫描线
    if (laserCloudScans[i].size() > 0) {
      // 将每个激光线的点云合并到输出点云中
      (*outcloud) += laserCloudScans[i];

      // 记录合并后的点云范围
      IndexRange range(cloudSize, 0);
      cloudSize += laserCloudScans[i].size();
      range.second = cloudSize > 0 ? cloudSize - 1 : 0;

      scanindices.push_back(range);
    }


  }



  // for (size_t j = 0 ;j < scanindices_l.size(); j++){
  //   cout << " 第 " << j << " 圈l的点云为从"<<  scanindices_l[j].first << " 到" << scanindices_l[j].second ;
  //   cout << " 第 " << j << " 圈r的点云为从"<<  scanindices_r[j].first << " 到" << scanindices_r[j].second << endl;
  // }

}


/**
 * @brief  建立扫描线的索引范围scanindices, 假设每一圈的点都没有过滤掉，每一圈 N 个点，则scanindices[0]=(0,N-1),scanindices[1]=(N-1,2N-2),scanindices[2]=(2N-2,3N-3)...

 *
 * @param incloud
 * @param outcloud
 * @param scanindices_l
 */
void Cloud_id::processByRing(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                  std::vector<std::vector<size_t>> &points_by_ring
                                  )
{
  // 获取输入点云的大小
  size_t cloudSize = incloud->points.size();
  std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(_nScanRings);
  std::vector<std::vector<std::pair<size_t,size_t>>> one_All(_nScanRings);
  points_by_ring.resize(_nScanRings);


  int scanID = 0;
  float Y_value = 0;

  // 从输入点云中根据 intensity 把不同的扫描线上的点云分类保存
  for (int i = 0; i < cloudSize; i++) {
    scanID = incloud->points[i].intensity;
    Y_value = incloud->points[i].y;


    // 与X轴的夹角(y轴正侧为正)
    float ori = std::atan2(incloud->points[i].y, incloud->points[i].x) * 180 / M_PI;

    // 原始分类点云
    one_All[scanID].push_back(std::make_pair(i,ori));

  }


  for(size_t i=0;i < one_All.size(); ++i){
    std::sort(one_All[i].begin(),
         one_All[i].end(),
         [](const std::pair<size_t, size_t>& a, const std::pair<size_t, size_t>& b){
            // 按照第一个元素升序排序
            return a.second < b.second;
          }
    );



    for(size_t j = 0;j < one_All[i].size(); ++j){
      points_by_ring[i].push_back(one_All[i][j].first);
    }

  }


}






/**
 * @brief 激光雷达按照时间顺序排列点云的情况下，雷达扫描线从上到下顺序扫描,更改强度给与ID
 *
 * @param incloud
 * @param outcloud
 */
void Cloud_id::processByOri(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud) {
  int scanID = 0;
  pcl::PointXYZI point;
  for (size_t i = 1; i < incloud->points.size(); i++) {
    // 跳过空值点
    if (!pcl_isfinite(incloud->points[i].x) ||
        !pcl_isfinite(incloud->points[i].y) ||
        !pcl_isfinite(incloud->points[i].z)) {
      continue;
    }

    // 跳过零值点
    if (incloud->points[i].x * incloud->points[i].x +
        incloud->points[i].y * incloud->points[i].y +
        incloud->points[i].z * incloud->points[i].z <
        0.0001) {
      continue;
    }

    float ori = std::atan2(incloud->points[i].y, incloud->points[i].x) * 180 / M_PI;
    float ori_pre = std::atan2(incloud->points[i - 1].y, incloud->points[i - 1].x) * 180 / M_PI;

    // 第一个点角度一般为正数，最后一个点角度一般为负数（除开及其特殊的情况），所以
    if (ori < 0) {
      ori += 360;
    }

    if (ori_pre < 0) {
      ori_pre += 360;
    }

    if (abs(ori - ori_pre) > 250) {
      scanID += 1;
    }


    if (scanID < getNumberOfScanRings()) {
      point = incloud->points[i];
      point.intensity = scanID;

      outcloud->points.push_back(point);
    }
  }
}

/**
 * @brief 激光雷达按照时间顺序排列点云的情况下，雷达扫描线从上到下顺序扫描,更改强度给与ID
 *
 * @param incloud
 * @param outcloud
 */
void Cloud_id::processByOri_XYZI2XYZIRT(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                        pcl::PointCloud<PointXYZIRT>::Ptr outcloud) {
  int scanID = 0;
  PointXYZIRT point;
  for (size_t i = 1; i < incloud->points.size(); i++) {
    // 跳过空值点
    if (!pcl_isfinite(incloud->points[i].x) ||
        !pcl_isfinite(incloud->points[i].y) ||
        !pcl_isfinite(incloud->points[i].z)) {
      continue;
    }

    // 跳过零值点
    if (incloud->points[i].x * incloud->points[i].x +
        incloud->points[i].y * incloud->points[i].y +
        incloud->points[i].z * incloud->points[i].z <
        0.0001) {
      continue;
    }

    float ori = std::atan2(incloud->points[i].y, incloud->points[i].x) * 180 / M_PI;
    float ori_pre = std::atan2(incloud->points[i - 1].y, incloud->points[i - 1].x) * 180 / M_PI;

    // 第一个点角度一般为正数，最后一个点角度一般为负数（除开及其特殊的情况），所以
    if (ori < 0) {
      ori += 360;
    }

    if (ori_pre < 0) {
      ori_pre += 360;
    }

    if (abs(ori - ori_pre) > 250) {
      scanID += 1;
    }


    if (scanID < getNumberOfScanRings()) {
      point.x = incloud->points[i].x;
      point.y = incloud->points[i].y;
      point.z = incloud->points[i].z;
      point.intensity = incloud->points[i].intensity;
      point.ring  = scanID;
      outcloud->points.push_back(point);
    }
  }
}



}
