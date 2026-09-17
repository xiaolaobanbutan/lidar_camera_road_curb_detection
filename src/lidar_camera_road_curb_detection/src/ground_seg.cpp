#include "lidar_camera_road_curb_detection/ground_seg.hpp"


namespace CurbDetection{

Ground_Seg::Ground_Seg(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, ransacMsg ran)
{
    _threshold = ran.segThres;
    _cloudptrlist.resize(2); // 更改RANSAC范围这里也要改
    // _cloudptrlist.resize(6); // 更改RANSAC范围这里也要改
    // _cloudptrlist和_cloud_list初始化
#pragma omp parallel for schedule(runtime)
    for (size_t i = 0; i < _cloudptrlist.size(); ++i) {
      _cloudptrlist[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }


  // 分段进行平面分割
  for (int i = 0; i < incloud->points.size(); ++i) {
    if (incloud->points[i].x <= 15 && incloud->points[i].x >= 0 &&
        abs(incloud->points[i].y) < 30) {
      _cloudptrlist[0]->points.push_back(incloud->points[i]);
    } // _cloudptrlist[0]: x 范围在 [0, 15]，y 范围在 [-30, 30] 的部分


    if (incloud->points[i].x > 15 && incloud->points[i].x <= 40 &&
        abs(incloud->points[i].y) < 30) {
      _cloudptrlist[1]->points.push_back(incloud->points[i]);
    } // _cloudptrlist[2]: x 范围在 (15, 30]，y 范围在 [-30, 30] 的部分

//==============//

    // if (incloud->points[i].x <= 40 && incloud->points[i].x >= 30 &&
    //     abs(incloud->points[i].y) < 30) {
    //   _cloudptrlist[2]->points.push_back(incloud->points[i]);
    // } // _cloudptrlist[1]: x 范围在 [30, 40]，y 范围在 [-30, 30] 的部分


    // if (incloud->points[i].x >= -15 && incloud->points[i].x < 0 &&
    //     abs(incloud->points[i].y) < 30) {
    //   _cloudptrlist[3]->points.push_back(incloud->points[i]);
    // } // _cloudptrlist[3]: x 范围在 [-15, 0)，y 范围在 [-30, 30] 的部分

    // if (incloud->points[i].x  >= -30 && incloud->points[i].x < -15 &&
    //     abs(incloud->points[i].y) < 30) {
    //   _cloudptrlist[4]->points.push_back(incloud->points[i]);
    // } // _cloudptrlist[4]: x 范围在 (-30, -15]，y 范围在 [-30, 30] 的部分

    // if (incloud->points[i].x <= -30 && incloud->points[i].x >= -40 &&
    //     abs(incloud->points[i].y) < 30) {
    //   _cloudptrlist[5]->points.push_back(incloud->points[i]);
    // } // _cloudptrlist[5]: x 范围在 [-40, -30)，y 范围在 [-30, 30] 的部分
  }

}

/**
 * @brief ExtractIndices设置删除点
 *
 * @param outCloud 输出点云
 * @param inputCloud 输入点云
 * @param Indices 索引点云
 * @param setNeg 设置为 true 表示保留索引的点，false 表示删除索引的点
 */
void Ground_Seg::extract_Ground(pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
                                pcl::PointIndices::Ptr Indices, bool setNeg) {
  pcl::ExtractIndices<pcl::PointXYZI> extract;

  // 设置为 true 表示提取（保留）指定索引的点
  // 设置为 false 表示排除（删除）指定索引的点
  extract.setNegative(setNeg);
  extract.setInputCloud(inputCloud);
  extract.setIndices(Indices);
  extract.filter(*outCloud);
}


/**
 * @brief  使用 RANSAC 方法拟合平面
 *
 * @param cloud 输入点云
 * @param coefficients 分割出的平面参数,如ax + by + cz + d = 0
 * @param planeIndices 分割平面点云的序号
 */
void Ground_Seg::RANSACCloud_Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                        pcl::PointIndices::Ptr planeIndices,
                        pcl::ModelCoefficients::Ptr coefficients) {
  pcl::SACSegmentation<pcl::PointXYZI> segmentation;  // 创建 SACSegmentation 实例
  segmentation.setInputCloud(cloud);                  // 设置输入点云
  segmentation.setModelType(pcl::SACMODEL_PLANE);     // 设置模型类型为平面
  segmentation.setMethodType(pcl::SAC_RANSAC);        // 设置方法类型为 RANSAC
  segmentation.setDistanceThreshold(_threshold);             // 设置距离阈值
  segmentation.setMaxIterations(200);                 // 设置了 RANSAC 迭代的最大次数
  // segmentation.setOptimizeCoefficients(true);         // 设置了是否对估计的模型系数进行优化

  // planeIndices 中包含被分割出的平面上的点的索引
  // coefficients 中包含平面模型的系数
  segmentation.segment(*planeIndices, *coefficients);
}






/**
 * @brief 综合处理模块
 *
 * @param groundpoints
 * @param non_groundpoints
 */
void Ground_Seg::ground_proess(pcl::PointCloud<pcl::PointXYZI>::Ptr groundpoints,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr non_groundpoints
                               ){
  float start = ros::Time::now().toSec();


  for (size_t i = 0; i < _cloudptrlist.size(); ++i) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_i(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_no_i(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    RANSACCloud_Plane(_cloudptrlist[i], indices, model);
    extract_Ground(ground_i, _cloudptrlist[i], indices, false);
    extract_Ground(ground_no_i, _cloudptrlist[i], indices, true);

    *groundpoints += (*ground_i);
    *non_groundpoints += (*ground_no_i);
  }

  float end = ros::Time::now().toSec();
}
}
 // namespace CurbDetection
