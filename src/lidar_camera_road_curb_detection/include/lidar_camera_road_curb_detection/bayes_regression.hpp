#ifndef BAYES_REGRESSION_H
#define BAYES_REGRESSION_H


#define MLPACK_PRINT_INFO
#define MLPACK_PRINT_WARN
#include <mlpack.hpp>
#include <mlpack/core.hpp>
#include <armadillo>
#include <mlpack/methods/bayesian_linear_regression.hpp>
#include "lidar_camera_road_curb_detection/core_head.hpp"
// 修改为
// #define MLPACK_PRINT_INFO
// #define MLPACK_PRINT_WARN
// #include <mlpack/core.hpp>  // 核心功能
// #include <mlpack/methods/bayesian_linear_regression/bayesian_linear_regression.hpp> // 贝叶斯回归
// #include <armadillo>
// #include "lidar_camera_road_curb_detection/core_head.hpp"
namespace CurbDetection {

class Bayes_Regression {
public:
  Bayes_Regression(pcl::PointCloud<pcl::PointXYZI>::Ptr candidatePoints,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr leftInitialPoints,
                  float rmseThres,
                  float meanThres);

  std::vector<int> vectors_difference(std::vector<int> v1, std::vector<int> v2);

  void process(pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud);

  void initialTrainData();

void transform_mat(std::vector<int> _Index,
              arma::mat& _x,
              arma::rowvec& _y
          );

private:
  pcl::PointCloud<pcl::PointXYZI> _candidatePoints; // 输入特征点候选点(已划分好网格ID)
  pcl::PointCloud<pcl::PointXYZI> _leftInitialPoints; // RANSAC处理的候选点(已划分好网格ID)
  //    pcl::PointCloud<pcl::PointXYZI> _rightInitialPoints;
  std::vector<Eigen::VectorXd> _xLeft; //存储RANSAC认为的边缘点坐标x
  std::vector<Eigen::VectorXd> _yLeft; //存储RANSAC认为的边缘点坐标y

  std::vector<Eigen::VectorXd> _remainX; //存储RANSAC认为的非边缘点云坐标x
  std::vector<Eigen::VectorXd> _remainY; //存储RANSAC认为的非边缘点云坐标y
  std::vector<int> _leftIndex; // vector格式，储存点云ID
  //    vector<int> _rightIndex;
  std::vector<int> _remainIndex; // 差集，存储RANSAC认为的非边缘点的ID
  std::vector<int> _candidateIndex;  // vector格式，储存点云ID

  arma::mat xTrain_;
  arma::rowvec yTrain_;

  arma::mat xRemain_;
  arma::rowvec yRemain_;

  float _meanThres;
  float _rmseThres;

  // 创建 BayesianLinearRegression 模型的参数
  bool centerData = true; // 是否对数据进行中心化。
  bool scaleData = true; // 是否对数据进行标准化。
  size_t maxIterations = 30; // 最大迭代次数。
  double tolerance = 1e-4; // 收敛容差。

  // struct Params {
  //   struct kernel_exp {
  //     BO_PARAM(float, sigma_sq, 38.1416);
  //     BO_PARAM(float, l, 16.1003);
  //     //            BO_PARAM(double, noise,0.0039);
  //   };
  //   struct kernel : public defaults::kernel {};
  //   struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {};
  //   struct opt_rprop : public defaults::opt_rprop {};
  // };

};
} // namespace CurbDetection

#endif
