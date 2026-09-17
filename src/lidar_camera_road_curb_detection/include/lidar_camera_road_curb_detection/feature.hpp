#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include "lidar_camera_road_curb_detection/core_head.hpp"

namespace CurbDetection{

class Feature_Point{
public:
    // 初始化所有参数
    Feature_Point(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                std::vector<IndexRange> scanIndices,
                const featurePointsMsg &fpMsg);

    void PT_Curvatures(int K_range );

    float compute_Horizon_Diff(int index, int region);

    float compute_Horizon_Diff_32c(int ns,int index, int region);

    float compute_Height_Sigma(int index, int region);

    float compute_Height_Sigma_32c(int ns, int index, int region);

    float Region_MaxZ(int j);

    float Region_MaxZ_32c(int ns,int j, int region);

    float Region_MinZ(int j);

    float Region_MinZ_32c(int ns,int j, int region);

    float calc_Point_Distance(const pcl::PointXYZI &p);

    std::vector<int> Vectors_Intersection(std::vector<int> v1,std::vector<int> v2);

    void extract_Points(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                        boost::shared_ptr<std::vector<int>> indices,
                        bool setNeg  = false);



    void extract_Features(pcl::PointCloud<pcl::PointXYZI>::Ptr &feature_points);

    // void extract_Features_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points);

    void extract_Features_XYZIRT(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vector,pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points);

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _incloud;
    scanIndices _scanindices;
    int _Height_Region;
    float _Height_SigmaThres;
    float _Height_MaxThres;
    float _Height_MinThres;
    float _Curvature_Thres;
    float _allCurvates_H;
    int _Curvature_Region;

    float _Distance_HorizonThres;
    float _Distance_VerticalThres;

    float _Angular_Res;

    int _Test_Region;
    float _Test_Thres;


    std::vector<int> _HeightPointsIndex;

    std::vector<std::vector<int>> _HeightPointsIndex_XYZIRT;

    std::vector<int> _CurvaturePointsIndex;

    std::vector<std::vector<int>> _CurvaturePointsIndex_XYZIRT;

    std::vector<int> _DistanceVerticlePointsIndex;

    std::vector<int> _DistanceHorizonPointsIndex;

    std::vector<int> _TestIndex;

    std::vector<std::vector<int>> _TestIndex_XYZIRT;

    std::vector<int> _INDEX;

    std::vector<std::vector<int>> _INDEX_XYZIRT;









    std::vector<std::vector<int>> _Height_Points_Index;

    std::vector<std::vector<int>> _Curvature_PointsIndex;

    std::vector<std::vector<int>> _Distance_VerticlePointsIndex;

    std::vector<std::vector<int>> _Distance_HorizonPointsIndex;

    std::vector<std::vector<int>> _Index;

    std::vector<PCURVATURE> _allCurvates;

    std::vector<int> _AllCurvates;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _points_ring_list;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _feature_points_list;

    bool _use_verticle;
    bool _use_horizon;
    bool _use_anglehorizon;
    };



}


#endif
