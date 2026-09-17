#ifndef DISTINGUISH_H
#define DISTINGUISH_H

#include "lidar_camera_road_curb_detection/core_head.hpp"



namespace CurbDetection{
class Distinguish_Road{
public:
    // 初始化函数
    Distinguish_Road(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                judgemsg JUDGE_MSG);

    // 按照角分辨率分成扇形区域，将扫描线转换为光束
    void Line_Proess_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,std::vector<std::vector<size_t>> &reslut);

    // 过滤同一Ring上距离参差不齐的点
    void Distance_on_Ring(std::vector<std::vector<size_t>> &points_by_ring);

    // 对于点云进行过滤，去除不符合条件的点云
    std::vector<std::vector<size_t>> Hight_Proess_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,std::vector<std::vector<size_t>> _input_indices);

    // 获取道路方向的角度
    float Road_DirecTon_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,std::vector<std::vector<size_t>> input_indices);

    // 计算点到原点的距离
    float calcPointDistance(const pcl::PointXYZI &p);

    // 汇总函数
    void Distinguish_Proess_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud,float &anglus_,std::vector<std::vector<size_t>> &_orgin_indices);




    // 根据数字在数组中的索引把扫描线转换为光束（废弃）
    std::vector<std::vector<size_t>> Line_Proess(scanIndices scan_indices);
    std::vector<std::vector<size_t>> Line_Proess_r(scanIndices scan_indices);

    std::vector<std::vector<size_t>> Hight_Proess(std::vector<std::vector<size_t>> _input_indices);

    std::vector<size_t> Road_DirecTon(std::vector<std::vector<size_t>> input_indices_l,std::vector<std::vector<size_t>> input_indices_r);

    void Angle_test(std::vector<std::vector<size_t>> _input_indices);


    void Distinguish_Proess(pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr flat_cloud,
                       scanIndices scanindices,
                       scanIndices scanindices_l,
                       scanIndices scanindices_r
                       );




    static float _pre_degrees;   // 前一次的角度
    static int _degrees_numb;    // 连续角度变化的次数

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _incloud;
    scanIndices _scan_indices;
    scanIndices _scan_indices_l;
    scanIndices _scan_indices_r;
    scanIndices _with_out_up_car_indices;
    scanIndices _with_out_up_car_indices_r;
    scanIndices _with_out_up_car_indices_l;

    std::vector<std::vector<size_t>> _points_by_ring;
    std::vector<std::vector<size_t>> _points_by_ring_l;
    std::vector<std::vector<size_t>> _points_by_ring_r;
    std::vector<std::vector<size_t>> _line_indices;
    std::vector<std::vector<size_t>> _line_indices_l;
    std::vector<std::vector<size_t>> _line_indices_r;
    std::vector<std::vector<size_t>> _Hight_indices;



    std::vector<int> _slope_road;
    std::vector<int> _flat_road;

    int _angular_ares_numb;

    int _MaxSacnID;
    // int _ID_test;
    int _MinSacnID;
    float _MinAnglus;
    float _Max_Hight_Cost;
    float _Max_Angle_Cost;
    float _Max_XY_Cost;
    float _angularRes;
    float _Vertical_Res;
    float _Ring_diff;
    bool _Is_XYZIRT_POINTS;
    int _nScanRings;


};
}


#endif // DISTINGUISH_H
