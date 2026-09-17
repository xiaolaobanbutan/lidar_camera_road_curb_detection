#include "lidar_camera_road_curb_detection/boundarypoints.hpp"
#ifdef ROAD_CURB_HAVE_MLPACK
#include "lidar_camera_road_curb_detection/bayes_regression.hpp"
#endif
// #include <thread>
// #include <future>  // 添加这一行
// 使用OpenMP并行化
#include <omp.h>
double sum1=0.0,sum2=0.0,sum3=0.0;
namespace CurbDetection {

BoundaryPoints::BoundaryPoints(pcl::PointCloud<pcl::PointXYZI> &incloud,
                               const cloud_msg &cmMsg,
                               const boundaryPointsMsg &bpMsg) {
  _cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  *_cloud = incloud;
  _gridNum = bpMsg.gridNum;
  _gridRes = bpMsg.gridRes;
  _rmseThres = bpMsg.rmseThres;
  _meanThres = bpMsg.meanThres;
  _curveFitThres = bpMsg.curveFitThres;
  _use_curve_fit = bpMsg.useCurveRansac;
  _cmMsg = cmMsg;
  // Leftcloud_initial_pub_out  = nh1.advertise<sensor_msgs::PointCloud2>("/Leftcloud_initial_out",10);
  // Rightcloud_initial_pub_out  = nh1.advertise<sensor_msgs::PointCloud2>("/Rightcloud_initial_out",10);

}



/**
 * @brief 根据提供的索引从输入点云中提取子集
 *
 * @param incloud
 * @param indices
 * @param outcloud
 */
void BoundaryPoints::extractPointCloud(pcl::PointCloud<pcl::PointXYZI> &incloud,
                                       pcl::PointIndicesPtr indices,
                                       pcl::PointCloud<pcl::PointXYZI> &outcloud) {

  //    boost::shared_ptr<vector<int> > indice_=boost::make_shared<vector<int>
  //    >(indices);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
  //    pcl::PointCloud<PointT>::Ptr cloudIn(new pcl::PointCloud<PointT>);
  //    *cloudIn=_completeCloud;
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setNegative(false);
  extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(incloud));
  extract.setIndices(indices);
  extract.filter(*cloudOut);
  outcloud = *cloudOut;
}

/**
 * @brief 使用 RANSAC（随机抽样一致性）对输入点云拟合线模型
 *
 * @param incloud
 * @param indices
 */
void BoundaryPoints::lineFitRansac(pcl::PointCloud<pcl::PointXYZI> &incloud,
                                   pcl::PointIndices &indices) {
  pcl::ModelCoefficients _model;
  pcl::PointIndices _indices;
  Eigen::Vector3f axis(1, 0, 0);
  //    pcl::ModelCoefficients model;
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  // seg.setAxis(axis);
  // seg.setEpsAngle(0.35);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(_curveFitThres);
  seg.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(incloud));
  seg.segment(_indices, _model);
  indices = _indices;
}

/**
 * @brief 使用统计离群值移除，基于均值和标准差过滤点
 *
 * @param incloud
 * @param meanK
 * @param pointindices
 * @param stdThreshold
 */
void BoundaryPoints::statisticalFilter_indces(pcl::PointCloud<pcl::PointXYZI> incloud, int meanK,
                                              pcl::PointIndicesPtr pointindices,
                                              double stdThreshold) {
  std::vector<int> indices;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(incloud));
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stdThreshold);
  sor.setKeepOrganized(false);
  // boost::make_shared<pcl::PointIndices>(_pointIndices)=
  // sor.getRemovedIndices();
  sor.filter(indices);
  // sor.filter(_indices);
  for (int i = 0; i < indices.size(); ++i) {
    pointindices->indices.push_back(indices[i]);
  }
}

/**
 * @brief 使用模型平面的系数将输入点云投影到平面上
 *
 * @param incloud
 * @param outcloud
 */
void BoundaryPoints::pointcloud_projection(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                           pcl::PointCloud<pcl::PointXYZI> &outcloud) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = 1;
  coefficients->values[1] = 0;
  coefficients->values[2] = 0;
  coefficients->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(incloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(outcloud);
}

/**
 * @brief 使用 RANSAC 对输入点云拟合曲线模型，然后利用最优模型对点云进行过滤，只保留符合模型的内点。
 *
 * @param incloud
 * @param residualThreshold
 * @param outcloud
 */
void BoundaryPoints::ransac_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                                  float residualThreshold,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud) {
  srand(time(NULL)); // time(NULL)获取当前时间,返回一个长整形数值
  //-------------------------------------------------------------- make sample
  // 点云转数组

  std::vector<double> x(incloud->points.size());
  std::vector<double> y(incloud->points.size());
  int nData = incloud->points.size();

  if (nData == 0 || nData == 1 || nData == 2)
    return;

  for (size_t i = 0; i < x.size(); i++) {
    x[i] = incloud->points[i].x;
    y[i] = incloud->points[i].y;
  }
  //-------------------------------------------------------------- build matrix
  // 新建矩阵A、B
  cv::Mat A(nData, 3, CV_64FC1);
  cv::Mat B(nData, 1, CV_64FC1);

  for (size_t i = 0; i < nData; i++) {
    A.at<double>(i, 0) = x[i] * x[i];
    A.at<double>(i, 1) = x[i];
    A.at<double>(i, 2) = 1.0;
    B.at<double>(i, 0) = y[i];
  }

  //-------------------------------------------------------------- RANSAC
  // fitting
  int N = 300;                  // 迭代次数
  double T = residualThreshold; // 拟合残差阈值

  int n_sample = 3;
  int max_cnt = 0;
  cv::Mat best_model(3, 1, CV_64FC1);

  for (int i = 0; i < N; i++) {
    // 随机抽样3点
    int k[3] = {
        -1,
    };

    k[0] = floor((rand() % nData + 1)) + 1;

    do {
      k[1] = floor((rand() % nData + 1)) + 1;
    } while (k[1] == k[0] || k[1] < 0);

    do {
      k[2] = floor((rand() % nData + 1)) + 1;
    } while (k[2] == k[0] || k[2] == k[1] || k[2] < 0);

    // printf("random sample : %d %d %d\n", k[0], k[1], k[2]);

    // 模型估计
    cv::Mat AA(3, 3, CV_64FC1);
    cv::Mat BB(3, 1, CV_64FC1);
    for (int j = 0; j < 3; j++) {
      AA.at<double>(j, 0) = x[k[j]] * x[k[j]];
      AA.at<double>(j, 1) = x[k[j]];
      AA.at<double>(j, 2) = 1.0;
      BB.at<double>(j, 0) = y[k[j]];
    }

    cv::Mat AA_pinv(3, 3, CV_64FC1);
    invert(AA, AA_pinv, cv::DECOMP_SVD); //求AA逆矩阵

    cv::Mat X = AA_pinv * BB;

    // 评价
    if (X.at<double>(0) < 0.1) {
      cv::Mat residual(nData, 1, CV_64FC1);
      residual = cv::abs(B - A * X);
      int cnt = 0; //内点计数
      for (int j = 0; j < nData; j++) {
        double data = residual.at<double>(j, 0);

        if (data < T) {
          cnt++;
        }
      }

      if (cnt > max_cnt) //如果内点数量大于0,则作为best model
      {
        best_model = X;
        max_cnt = cnt;
      }
    }
  }

  //------------------------------------------------------------------- 可选择的
  // LS fitting
  cv::Mat residual = cv::abs(A * best_model - B);
  std::vector<int> vec_index; //存储模型内点索引
  for (int i = 0; i < nData; i++) {
    double data = residual.at<double>(i, 0);
    if (data < T) //如果残差小于阈值则作为内点
    {
      vec_index.push_back(i);
    }
  }
  // AINFO << "done LS fitting" << endl;

  if (vec_index.size() == 0)
    return;

  cv::Mat A2(vec_index.size(), 3, CV_64FC1); //存储所有内点
  cv::Mat B2(vec_index.size(), 1, CV_64FC1);

  for (int i = 0; i < vec_index.size(); i++) {
    A2.at<double>(i, 0) = x[vec_index[i]] * x[vec_index[i]];
    A2.at<double>(i, 1) = x[vec_index[i]];
    A2.at<double>(i, 2) = 1.0;
    B2.at<double>(i, 0) = y[vec_index[i]];
  }

  cv::Mat A2_pinv(3, vec_index.size(), CV_64FC1);

  invert(A2, A2_pinv, cv::DECOMP_SVD);

  cv::Mat X = A2_pinv * B2; //利用所有内点再次进行拟合得到优化后的模型系数

  cv::Mat residual_opt = cv::abs(A * X - B);
  std::vector<int> vec_index_opt; //存储模型内点索引

  for (int i = 0; i < nData; i++) {
    double data = residual_opt.at<double>(i, 0);
    if (data < T) //如果残差小于阈值则作为内点
    {
      outcloud->points.push_back(incloud->points[i]);
    }
  }

}


/**
 * @brief 整体处理函数，接受障碍物点云，将其聚类为左右两部分，执行距离过滤，并应用高斯过程和其他过滤技术
 *
 * @param obstacleCloud
 * @param clusterCloud
 * @param road_central_line
 */
void BoundaryPoints::process(pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_left,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_right,
                             std::vector<visualization_msgs::Marker> &road_central_line) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2D(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFilted(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudLeft2D(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudRight2D(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudLeftFiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudRightFiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointIndices::Ptr leftIndices(new pcl::PointIndices);
  pcl::PointIndices::Ptr rightIndices(new pcl::PointIndices);
  pcl::PointIndices::Ptr cloudIndices(new pcl::PointIndices);

  // 新建左右两个点云的集合
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterPtrLR(2);
  for (int i = 0; i < clusterPtrLR.size(); ++i) {
    clusterPtrLR[i] = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  // 非地面点云移除车顶干扰点
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < obstacleCloud->points.size(); ++i) {
    if ((pow(obstacleCloud->points[i].x, 2) + pow(obstacleCloud->points[i].y, 2)) >= 4) {
      obstacleCloudFiltered->points.push_back(obstacleCloud->points[i]);
    }
  }


  // 根据非地面点云获取道路中心线，进行粗分割,
  RoadSegmentation mycluster(obstacleCloudFiltered);
  mycluster.process(_cloud, clusterPtrLR, road_central_line);
  // cout << "left candidate points is" << clusterPtrLR[0]->points.size() << endl;
  // cout << "right candidate points is " << clusterPtrLR[1]->points.size()
  //       << endl;



  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_distancefiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_distanceleftfiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_distancerightfiltered(new pcl::PointCloud<pcl::PointXYZI>);

  #pragma omp parallel sections
  {
    #pragma omp section
    {
      // 划分左侧道路特征点网格，获取每个网格中最靠近X轴的点的id
      GridMap my_mapL(clusterPtrLR[0], _gridRes, _gridNum);
      my_mapL.distanceFilterByCartesianGrid(pointcloud_distanceleftfiltered, true);
      for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i) {
        pointcloud_distanceleftfiltered->points[i].intensity = i;
      }
    }
    #pragma omp section
    {
      // 划分右侧道路特征点网格，获取每个网格中最靠近X轴的点的id
      GridMap my_mapR(clusterPtrLR[1], _gridRes, _gridNum);
      my_mapR.distanceFilterByCartesianGrid(pointcloud_distancerightfiltered,false);
      for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i) {
        pointcloud_distancerightfiltered->points[i].intensity = i;
      }
    }
  }
  // // 集合到一起
  // *Cloud_left = *pointcloud_distanceleftfiltered;
  // *Cloud_right = *pointcloud_distancerightfiltered;

  // cout << "distance filter left is "
  //       << pointcloud_distanceleftfiltered->points.size() << endl;
  // cout << "distance filter right is "
  //       << pointcloud_distancerightfiltered->points.size() << endl;



  // //将道路最近边界点分成前后两段用于拟合

  // // pcl::PointCloud<pcl::PointXYZI>::Ptr LeftFrontcloud(new pcl::PointCloud<pcl::PointXYZI>);
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr LeftRearcloud(new pcl::PointCloud<pcl::PointXYZI>);
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr RightFrontcloud(new pcl::PointCloud<pcl::PointXYZI>);
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr RightRearcloud(new pcl::PointCloud<pcl::PointXYZI>);

  // // for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i)
  // // {
  // //     if (pointcloud_distanceleftfiltered->points[i].x >= 0)
  // //     {
  // //         LeftFrontcloud->points.push_back(pointcloud_distanceleftfiltered->points[i]);
  // //     }
  // //     else
  // //     {
  // //         LeftRearcloud->points.push_back(pointcloud_distanceleftfiltered->points[i]);
  // //     }
  // // }

  // // for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i)
  // // {
  // //     if (pointcloud_distancerightfiltered->points[i].x >= 0)
  // //     {
  // //         RightFrontcloud->points.push_back(pointcloud_distancerightfiltered->points[i]);
  // //     }
  // //     else
  // //     {
  // //         RightRearcloud->points.push_back(pointcloud_distancerightfiltered->points[i]);
  // //     }
  // // }

  // // pcl::PointCloud<pcl::PointXYZI>::Ptr LeftFront_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr LeftRear_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr RightFront_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  // // pcl::PointCloud<pcl::PointXYZI>::Ptr RightRear_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // // pcl::PointIndicesPtr LeftFront_indices(new pcl::PointIndices);
  // // pcl::PointIndicesPtr LeftRear_indices(new pcl::PointIndices);
  // // pcl::PointIndicesPtr RightFront_indices(new pcl::PointIndices);
  // // pcl::PointIndicesPtr RightRear_indices(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZI>::Ptr Leftcloud_initial(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr Rightcloud_initial(new pcl::PointCloud<pcl::PointXYZI>);

  // // if (_use_curve_fit)
  // // {
  // //     ransac_curve(LeftFrontcloud,_curveFitThres,LeftFront_filtered);
  // //     ransac_curve(LeftRearcloud,_curveFitThres,LeftRear_filtered);
  // //     ransac_curve(RightFrontcloud,_curveFitThres,RightFront_filtered);
  // //     ransac_curve(RightRearcloud,_curveFitThres,RightRear_filtered);
  // // }
  // // else
  // // {
  // //     lineFitRansac(*LeftFrontcloud,*LeftFront_indices);
  // //     extractPointCloud(*LeftFrontcloud,LeftFront_indices,*LeftFront_filtered);

  // //     lineFitRansac(*LeftRearcloud,*LeftRear_indices);
  // //     extractPointCloud(*LeftRearcloud,LeftRear_indices,*LeftRear_filtered);

  // //     lineFitRansac(*RightFrontcloud,*RightFront_indices);
  // //     extractPointCloud(*RightFrontcloud,RightFront_indices,*RightFront_filtered);

  // //     lineFitRansac(*RightRearcloud,*RightRear_indices);
  // //     extractPointCloud(*RightRearcloud,RightRear_indices,*RightRear_filtered);

  // // }

  // // *Leftcloud_initial=*LeftFront_filtered+*LeftRear_filtered;
  // // *Rightcloud_initial=*RightFront_filtered+*RightRear_filtered;


  //  使用 RANSAC 对对于最靠近x轴的点云拟合曲线模型，然后利用最优模型对点云进行过滤，只保留符合模型的内点
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      ransac_curve(pointcloud_distanceleftfiltered, _curveFitThres,Leftcloud_initial);
    }
    #pragma omp section
    {
      ransac_curve(pointcloud_distancerightfiltered, _curveFitThres,Rightcloud_initial);
    }
  }
  *Cloud_left = *Leftcloud_initial;
  *Cloud_right = *Rightcloud_initial;

  // // 使用高斯过程处理点云
  // GaussianProcess my_gauLeft(pointcloud_distanceleftfiltered, Leftcloud_initial,
  //                            _rmseThres, _meanThres);
  // my_gauLeft.process(clusterCloud[0]);
  // GaussianProcess my_gauRight(pointcloud_distancerightfiltered,
  //                             Rightcloud_initial, _rmseThres, _meanThres);
  // my_gauRight.process(clusterCloud[1]);

  // AINFO << "在高斯之前左点数量：” " << Leftcloud_initial->points.size()
  //       << endl;
  // AINFO << "在高斯之后左点数量： " << clusterCloud[0]->points.size()
  //       << endl;
  // AINFO << "在高斯之前右点数量："
  //       << Rightcloud_initial->points.size() << endl;
  // AINFO << "在高斯之后右点数量：" << clusterCloud[1]->points.size()
  //       << endl;
}

void BoundaryPoints::process_with_picture(
                             pcl::PointCloud<pcl::PointXYZI>::Ptr ALL_Cloud_left,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr ALL_Cloud_right,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_left,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_right,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr initial_Cloud_left,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr initial_Cloud_right,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr left_filtered,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr right_filtered,
                             visualization_msgs::Marker &road_central_line,
                             int Dir_Angle,
                             double &stime
                             ) {
  double start = ros::Time::now().toSec();
  // 获取点云时间戳
  // std::string in_cloud_frame_id_left = ALL_Cloud_left->header.frame_id;
  // std::string in_cloud_frame_id_right = ALL_Cloud_right->header.frame_id;

  // 新建左右两个点云的集合
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterPtrLR(2);
  for (int i = 0; i < clusterPtrLR.size(); ++i) {
    clusterPtrLR[i] = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  // // 新建左右两个点云的集合
  // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> VectorPtrLR(2);
  // for (int i = 0; i < VectorPtrLR.size(); ++i) {
  //   VectorPtrLR[i] = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  // }



  // 根据图片转换点云获取道路中心线，进行粗分割,
  RoadSegmentation mycluster(_cloud);
  mycluster.process_with_picture(_cloud, clusterPtrLR, road_central_line, Dir_Angle);
  // cout << "left  candidate points is " << clusterPtrLR[0]->points.size() << endl;
  // cout << "right candidate points is " << clusterPtrLR[1]->points.size() << endl;

  pcl::copyPointCloud(*clusterPtrLR[0],*ALL_Cloud_left);
  pcl::copyPointCloud(*clusterPtrLR[1],*ALL_Cloud_right);

  // pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_distancefiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_distanceleftfiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_distancerightfiltered(new pcl::PointCloud<pcl::PointXYZI>);

  // 网格划分
  // // 划分左侧道路特征点网格，获取每个网格中y值最小的点的id
  // GridMap my_mapL(clusterPtrLR[0], _gridRes, _gridNum);
  // my_mapL.distanceFilterByCartesianGrid(pointcloud_distanceleftfiltered, true);
  // for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i) {
  //   pointcloud_distanceleftfiltered->points[i].intensity = i;
  // }

  // // 划分右侧道路特征点网格，获取每个网格中y值最小的点的id
  // GridMap my_mapR(clusterPtrLR[1], _gridRes, _gridNum);
  // my_mapR.distanceFilterByCartesianGrid(pointcloud_distancerightfiltered,false);
  // for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i) {
  //   pointcloud_distancerightfiltered->points[i].intensity = i;
  // }
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      // ID线划分
      // 划分左侧道路特征点网格，获取每个网格中y值最小的点的id
      GridMap my_mapL(clusterPtrLR[0], _gridRes, _gridNum);
      my_mapL.distanceFilterByCartesianGrid_with_ID_scan(pointcloud_distanceleftfiltered, true);
      for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i) {
        pointcloud_distanceleftfiltered->points[i].intensity = i;
      }
    }
    #pragma omp section
    {
      // 划分右侧道路特征点网格，获取每个网格中y值最小的点的id
      GridMap my_mapR(clusterPtrLR[1], _gridRes, _gridNum);
      my_mapR.distanceFilterByCartesianGrid_with_ID_scan(pointcloud_distancerightfiltered,false);
      for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i) {
        pointcloud_distancerightfiltered->points[i].intensity = i;
      }
    }
  }
  // cout << "左侧网格点数量：" << pointcloud_distanceleftfiltered->points.size() << endl;
  // cout << "右侧网格点数量：" << pointcloud_distancerightfiltered->points.size() << endl;



  pcl::PointCloud<pcl::PointXYZI>::Ptr Leftcloud_initial(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr Rightcloud_initial(new pcl::PointCloud<pcl::PointXYZI>);


  // 使用 RANSAC 对输入点云拟合曲线模型，然后利用最优模型对点云进行过滤，只保留符合模型的内点



  // 在process_with_picture函数中
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      double start_l = ros::Time::now().toSec();
      // 左侧处理
      *left_filtered=*pointcloud_distanceleftfiltered;
      ransac_curve(pointcloud_distanceleftfiltered, _curveFitThres, Leftcloud_initial);

     for(int i=0;i<Leftcloud_initial->points.size();i++){
      // cout<<Leftcloud_initial->points[i].x<<" "<<Leftcloud_initial->points[i].y<<endl;
    }
      *initial_Cloud_left = *Leftcloud_initial;
#ifdef ROAD_CURB_HAVE_MLPACK
      Bayes_Regression my_gauLeft(pointcloud_distanceleftfiltered, Leftcloud_initial, _rmseThres, _meanThres);
      my_gauLeft.process(Cloud_left);
#else
      *Cloud_left = *Leftcloud_initial;
#endif
      double end_l = ros::Time::now().toSec();
      // cout << "\033[1m\033[36m" << "左侧边界线拟合耗时= " << end_l - start_l << " 秒. " << endl;
      sum1=end_l-start_l;
    }

    #pragma omp section
    {
      double start_r = ros::Time::now().toSec();

      // 右侧处理
      *right_filtered=*pointcloud_distancerightfiltered;
      ransac_curve(pointcloud_distancerightfiltered, _curveFitThres, Rightcloud_initial);
      for(int i=0;i<Rightcloud_initial->points.size();i++){
        // cout<<Rightcloud_initial->points[i].x<<" "<<Rightcloud_initial->points[i].y<<endl;
      }
      *initial_Cloud_right= *Rightcloud_initial;
#ifdef ROAD_CURB_HAVE_MLPACK
      Bayes_Regression my_gauRight(pointcloud_distancerightfiltered, Rightcloud_initial, _rmseThres, _meanThres);
      my_gauRight.process(Cloud_right);
#else
      *Cloud_right = *Rightcloud_initial;
#endif
      double end_r = ros::Time::now().toSec();
      // cout << "\033[1m\033[36m" << "右侧边界线拟合耗时= " << end_r - start_r << " 秒. " << endl;
      sum2=end_r-start_r;
    }
  }
  stime=sum1+sum2;
  // cout << "\033[1m\033[36m" << "stime= " << stime << " 秒. " << endl;
    // ransac_curve(pointcloud_distanceleftfiltered, _curveFitThres,Leftcloud_initial);
  // // 使用bayes处理点云
  // Bayes_Regression my_gauLeft(pointcloud_distanceleftfiltered, Leftcloud_initial,_rmseThres, _meanThres);
  // my_gauLeft.process(Cloud_left);

  // 发布左侧点云
  // sensor_msgs::PointCloud2 Leftcloud_initial_pub;
  // // pcl::toROSMsg(cloud_road_direction, road_direction);
  // pcl::toROSMsg(*Leftcloud_initial, Leftcloud_initial_pub);
  // Leftcloud_initial_pub.header.frame_id = in_cloud_frame_id_left;
  // Leftcloud_initial->clear();
  // // 发布右侧点云
  // sensor_msgs::PointCloud2 Rightcloud_initial_pub;
  // // pcl::toROSMsg(cloud_road_direction, road_direction);
  // pcl::toROSMsg(*Rightcloud_initial, Rightcloud_initial_pub);
  // Rightcloud_initial_pub.header.frame_id = in_cloud_frame_id_right;
  // Rightcloud_initial->clear();

  // Leftcloud_initial_pub_out.publish(Leftcloud_initial_pub);

  // Rightcloud_initial_pub_out.publish(Rightcloud_initial_pub);

  // ransac_curve(pointcloud_distancerightfiltered, _curveFitThres,Rightcloud_initial);

  // Bayes_Regression my_gauRight(pointcloud_distancerightfiltered,Rightcloud_initial, _rmseThres, _meanThres);
  // my_gauRight.process(Cloud_right);
  // cout << "左侧RANSAC数量：" << Leftcloud_initial->points.size() << endl;
  // cout << "右侧RANSAC数量：" << Rightcloud_initial->points.size() << endl;

  // 论文中说明，在使用高斯过程之前的情况，也就是图8和图9区别
  // *Cloud_left = *pointcloud_distanceleftfiltered;
  // *Cloud_right = *pointcloud_distancerightfiltered;


  double end = ros::Time::now().toSec();
  // sum1+=end-start;
  // std::cout<<"RANSAC和BAYES消耗总时间："<<sum1<<endl;

  // std::cout << "\033[1m\033[36m" << "使用RANSAC以及BAYES处理特征点消耗时间 = " << end - start << " 秒. " << endl;

  std::cout << "在贝叶斯回归之前左点数量： " << Leftcloud_initial->points.size()  << std::endl;

  std::cout << "在贝叶斯回归之后左点数量： " << Cloud_left->points.size() << std::endl;

  std::cout << "在贝叶斯回归之前右点数量： " << Rightcloud_initial->points.size() << std::endl;

  std::cout << "在贝叶斯回归之后右点数量： " << Cloud_right->points.size() << std::endl;

  std::cout << "在贝叶斯回归之后总点数量： " << Cloud_right->points.size()+Cloud_left->points.size() << std::endl;

}

}
