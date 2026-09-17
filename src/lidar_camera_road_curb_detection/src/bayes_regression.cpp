#include "lidar_camera_road_curb_detection/bayes_regression.hpp"

namespace CurbDetection {

/**
 * @brief 接受两个指向点云的智能指针 candidatePoints 和 leftInitialPoints，以及两个阈值 rmseThres 和 meanThres。
 * 在构造函数中，将参数存储在成员变量 _rmseThres 和 _meanThres 中，并对输入点云进行初始化和预处理。
 * 首先，将输入点云 candidatePoints 和 leftInitialPoints 深拷贝给类成员变量 _candidatePoints 和 _leftInitialPoints，
 * 然后计算点云的索引并存储在 _candidateIndex 和 _leftIndex 中。
 * 最后，通过调用 vectors_difference 函数计算剩余点云的索引并存储在 _remainIndex 中。
 *
 * @param candidatePoints 输入特征点候选点
 * @param leftInitialPoints RANSAC处理的候选点
 * @param rmseThres 未知参数
 * @param meanThres 未知参数
 */
Bayes_Regression::Bayes_Regression(pcl::PointCloud<pcl::PointXYZI>::Ptr candidatePoints,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr leftInitialPoints,
                                 float rmseThres,
                                 float meanThres)
    : _rmseThres(rmseThres), _meanThres(meanThres) {

  _candidatePoints = *candidatePoints;
  _leftInitialPoints = *leftInitialPoints;
  _candidateIndex.resize(_candidatePoints.size());
  _leftIndex.resize(_leftInitialPoints.size());


  // 把ID存到VECTOR中
  #pragma omp parallel for schedule(runtime)
  for (int i = 0; i < _candidatePoints.size(); ++i) {
    _candidateIndex[i] = (int)_candidatePoints[i].intensity;
  }

  #pragma omp parallel for schedule(runtime)
  for (int i = 0; i < _leftInitialPoints.size(); ++i) {
    _leftIndex[i] = (int)_leftInitialPoints[i].intensity;
  }

  // 根据ID取两个点云之间的差集
  _remainIndex = vectors_difference(_candidateIndex, _leftIndex);
}

void Bayes_Regression::transform_mat(std::vector<int> _Index,
              arma::mat& _x,
              arma::rowvec& _y
          )
{
  std::vector<double> xx(_Index.size());
  std::vector<double> yy(_Index.size());
    // 将数据存储到矩阵中

  #pragma omp parallel for schedule(runtime)
  for (size_t i = 0; i < _Index.size(); ++i)
  {
    // 获取当前数据点的特征值
    xx[i] = _candidatePoints[_Index[i]].x;
    yy[i] = _candidatePoints[_Index[i]].y;
  }

  // 要求一行N列
  _x = arma::conv_to<arma::rowvec>::from(xx);
  _y = arma::conv_to<arma::rowvec>::from(yy);
}


std::vector<int> Bayes_Regression::vectors_difference(std::vector<int> v1,
                                                     std::vector<int> v2) {
  std::vector<int> v;
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
  return v;
}

/**
 * @brief 这一步主要是格式的转换，由 pcl::PointCloud<pcl::PointXYZI>::Ptr 转 Eigen::VectorXd
 *
 */
void Bayes_Regression::initialTrainData() {
  // Train的大小由有道路点的网格的数量决定
  // xTrain_.resize(_leftIndex.size());
  // yTrain_.resize(_leftIndex.size());

  // 根据网格的ID，把对应网格内的点放进 Eigen::VectorXd 的矩阵中
  transform_mat(_leftIndex,xTrain_,yTrain_);

  transform_mat(_remainIndex,xRemain_,yRemain_);
}



/**
 * @brief 该过程类似于插值
 * 1、调用 initialTrainData 函数初始化训练数据
 * 2、由样本计算高斯核
 * 3、添加非道路点，计算高斯核，如果正确，作为新的道路点放进道路点样本中，转到 第2步
 * 4、直到所有的点都已经尝试过添加的道路点中
 *
 * @param clusterCloud
 */
void Bayes_Regression::process(pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud) {
  this->initialTrainData();

  int newNumber = _leftIndex.size();


  if (newNumber > 0) {
    // 由道路点计算，也就是样本
    mlpack::BayesianLinearRegression<> estimator(centerData,
                                                scaleData,
                                                maxIterations,
                                                tolerance);
    newNumber = 0;

    for (int i = 0; i < _remainIndex.size(); ++i) {
      estimator.Train(xTrain_,yTrain_);

      arma::mat xT_1;
      arma::mat yT_1;
      double rmse;

      std::vector<double> points_x;
      std::vector<double> points_y;

      points_x.push_back(_candidatePoints[_remainIndex[i]].x);
      points_y.push_back(_candidatePoints[_remainIndex[i]].y);

      xT_1 = arma::conv_to<arma::rowvec>::from(points_x);
      yT_1 = arma::conv_to<arma::rowvec>::from(points_y);

      rmse = estimator.RMSE(xT_1,yT_1);

      if (abs(rmse) < _rmseThres) {
        _leftIndex.push_back(_remainIndex[i]);
        xTrain_ = arma::join_rows(xTrain_, xT_1);
        yTrain_ = arma::join_rows(yTrain_, yT_1);
        newNumber += 1;
      }

      points_x.clear();
      points_y.clear();
    }
  }



  for (int i = 0; i < _leftIndex.size(); ++i) {
    clusterCloud->points.push_back(_candidatePoints[_leftIndex[i]]);
  }


  // std::cout << "bayes sucess" << endl;
}



}
