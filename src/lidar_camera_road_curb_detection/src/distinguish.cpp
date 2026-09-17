#include "lidar_camera_road_curb_detection/distinguish.hpp"

namespace CurbDetection {

// 初始化参数
Distinguish_Road::Distinguish_Road(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                     judgemsg JUDGE_MSG
                     ){
    _incloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _incloud = incloud;
    _MaxSacnID = JUDGE_MSG.MaxSacnID;
    _MinSacnID = JUDGE_MSG.MinSacnID;
    _MinAnglus = JUDGE_MSG.MinAnglus;
    _Max_Hight_Cost = JUDGE_MSG.Max_Hight_Cost;
    _Max_Angle_Cost = JUDGE_MSG.Max_Angle_Cost;
    _Max_XY_Cost = JUDGE_MSG.Max_XY_Cost;
    // _ID_test = JUDGE_MSG.ID_test;
    _angularRes = JUDGE_MSG.angularRes;
    _angular_ares_numb = ceil(360 / _angularRes); // 向上取整
    _Vertical_Res = JUDGE_MSG.Vertical_Res/JUDGE_MSG.nScanRings * PI / 180;  // 垂直度分辨率
    _Ring_diff = JUDGE_MSG.Ring_diff;
    _nScanRings = JUDGE_MSG.nScanRings;

}


/**
 * @brief 转化区间为具体的数值(从前向后)
 *
 * @param scan_indices
 * @return std::vector<std::vector<size_t>>
 */
std::vector<std::vector<size_t>> Distinguish_Road::Line_Proess(scanIndices scan_indices){

    // 计算最大组大小
    size_t maxGroupSize = 0;
    for (const auto& interval : scan_indices) {
        // 不允许为空或者为无穷大的数组
        if ((interval.second - interval.first) < 0 || (interval.second - interval.first) > 999999){
            continue;}
        maxGroupSize = std::max(maxGroupSize, interval.second - interval.first + 1);
    }

    std::vector<std::vector<size_t>> result(maxGroupSize);


    // 将每个数字按照在组中的位置索引分类存储到 result 中
    for (size_t i = 0; i < maxGroupSize; ++i) {
        for (const auto& interval : scan_indices) {
            size_t num = interval.first + i;
            if (num <= interval.second) {
                result[i].push_back(num);
            }
        }
    }

    return result;

};

/**
 * @brief 按照角分辨率分成扇形区域，将ring转换为光束
 *
 * @param incloud
 * @param result
 */
void Distinguish_Road::Line_Proess_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, std::vector<std::vector<size_t>> &result){

    //
    std::vector<std::vector<std::pair<size_t,size_t>>> one_Ring;
    one_Ring.resize(result.size());

    // 32c雷达点云从近及远，大致按射线排序
    for (int i = 0; i < incloud->points.size(); i++) {
        // 与X轴的夹角(y轴正侧为正)
        float degrees = atan2(incloud->points[i].y, incloud->points[i].x) * 180 / M_PI;

        // 将角度映射到[0, 180)范围内
        if (degrees < 0) {
            degrees += 180.0;
        }

        size_t groupIndex = floor(degrees / _angularRes);  // 向下取整

        one_Ring[groupIndex].push_back(std::make_pair(i,incloud->points[i].intensity));
    }


    // 根据ring重新排序
    for(size_t j = 0;j < one_Ring.size(); ++j){
        sort(one_Ring[j].begin(),
             one_Ring[j].end(),
             [](const std::pair<size_t, size_t>& a, const std::pair<size_t, size_t>& b){
                // 元素排序,从小到大
                return a.second < b.second;
            }
        );

        for(size_t p = 0;p < one_Ring[j].size(); ++p){
            result[j].push_back(one_Ring[j][p].first);
        }

    }





    // for  (int i = 0; i < result.size(); i++) {
    //     for (int j = 0; j < result[i].size(); j++) {
    //     cout << result[i][j] << endl;
    //     }

    // }


};






/**
 * @brief 转化区间为具体的数值(从后向前)
 *
 * @param scan_indices
 * @return std::vector<std::vector<size_t>>
 */
std::vector<std::vector<size_t>> Distinguish_Road::Line_Proess_r(scanIndices scan_indices){

    // 计算最大组大小
    size_t maxGroupSize = 0;
    for (const auto& interval : scan_indices) {
        if ((interval.second - interval.first) < 0 || (interval.second - interval.first) > 999999 ){
            continue;}
        maxGroupSize = std::max(maxGroupSize, interval.second - interval.first + 1);
    }


    std::vector<std::vector<size_t>> result(maxGroupSize);


    // 将每个数字按照在组中的位置索引分类存储到 result 中
    for (size_t i = 0; i < maxGroupSize; ++i) {
        for (const auto& interval : scan_indices) {
            size_t index = interval.second - i;
            if (index >= interval.first) {
                result[i].push_back(index);
            }
        }
    }

    return result;

};





std::vector<std::vector<size_t>> Distinguish_Road::Hight_Proess(std::vector<std::vector<size_t>> _input_indices){
    // 设置返回的索引,GroupSize表示有多少条光束
    size_t GroupSize = _input_indices.size();
    std::vector<std::vector<size_t>> result(GroupSize);
    for (size_t i = 0;i < GroupSize ; ++i){
        // 排除点数过少的子集
        if (_input_indices[i].size() <= 4) {
            result[i].push_back(NULL);
            continue;}

        float ID_p0 = 0 ;
        float ID_p1 = 0 ;
        float ID_p2 = 0 ;
        float Vertical_Cost = 0 ;
        float false_numb = 0;

        // 最近的雷达扫描线是64(63)线，最远的是1(0)线,并且不处理64(63)线以及(0)线
        for(size_t k = _input_indices[i].size()-1; k > 0; k--){
            if (k == _input_indices[i].size()-1){
                result[i].push_back(_input_indices[i][k]);
                continue;
            }


            ID_p0 = _input_indices[i][k-1]; // 远处点
            ID_p1 = _input_indices[i][k];
            ID_p2 = _input_indices[i][k+1]; // 近处点



            float x0 = _incloud->points[ID_p0].x;
            float x1 = _incloud->points[ID_p1].x;
            float x2 = _incloud->points[ID_p2].x;

            float y0 = _incloud->points[ID_p0].y;
            float y1 = _incloud->points[ID_p1].y;
            float y2 = _incloud->points[ID_p2].y;

            float z0 = _incloud->points[ID_p0].z;
            float z1 = _incloud->points[ID_p1].z;
            float z2 = _incloud->points[ID_p2].z;



            // 1、平滑度特征
            // 模的平方，三点之间距离不是一样长不具有代表性
            // float Cost = sqrt(pow(x0 + x2 - 2* x1, 2)+ pow(y0 + y2 - 2* y1, 2));

            // 一条直线上三点构成的两向量之间的夹角θ
            float Cost_up = (x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1);
            float Cost_down = sqrt( pow((x1 - x0), 2) + pow((y1 - y0), 2)) * sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
            float Cost = acos(abs(Cost_up / Cost_down)) * 180 / PI;

            // 2、平面距离特征，同一光束不同点之间的距离
            float ID_p1_distance = sqrt(pow(x1, 2)+pow(y1, 2)+pow(z1, 2));
            float Cost_distance = sqrt(2*pow(ID_p1_distance, 2) -2*ID_p1_distance*ID_p1_distance*cos(_Vertical_Res));
            float True_distance = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2)+ pow((z1 - z0), 2));

            // 3、一条直线上三点构成的两向量与Z轴之间的夹角,只有在光束垂直打在墙壁上效果才好
            float Angle_Cost1  = acos(abs(z2 - z1)/sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)+pow((z2 - z1), 2)))* 180 / PI;
            float Angle_Cost2  = acos(abs(z0 - z1)/sqrt(pow((x0 - x1), 2) + pow((y0 - y1), 2)+pow((z0 - z1), 2)))* 180 / PI;
            float Angle_Cost = (Angle_Cost1 + Angle_Cost2) /2;

            if ( Cost >= _MinAnglus
                || Cost_distance >= True_distance
                || Angle_Cost <= _Max_Angle_Cost
                || false_numb >= _input_indices[i].size()/3){
                // false_numb++ ;
                continue;}

            result[i].push_back(ID_p1);
        }



        if (result[i].size() <= 3) {
            std::fill(result[i].begin(), result[i].end(), 0);
        }

    }


    return result;
};


/**
 * @brief 对于点云进行过滤，去除不符合条件的点云
 *
 * @param incloud
 * @param _input_indices
 * @return std::vector<std::vector<size_t>>
 */
std::vector<std::vector<size_t>> Distinguish_Road::Hight_Proess_32c(
    pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
    std::vector<std::vector<size_t>> _input_indices){

    // 设置返回的索引,GroupSize表示有多少条光束
    size_t GroupSize = _input_indices.size();
    std::vector<std::vector<size_t>> result(GroupSize);

    for (size_t i = 0;i < GroupSize ; ++i){
        // 排除点数过少的子集
        if (_input_indices[i].size() <= 4) {
            result[i].push_back(NULL);
            continue;}

        float ID_p0 = 0 ;
        float ID_p1 = 0 ;
        float ID_p2 = 0 ;

        // 从最近点往远处
        for(size_t k = 0; k < _input_indices[i].size(); k++){

            if (k == _input_indices[i].size()-1){
                // result[i].push_back(_input_indices[i][k]);
                continue;
            }

            if (k == 0){
                result[i].push_back(_input_indices[i][k]);
                continue;
            }

            ID_p0 = _input_indices[i][k-1]; // 近处点
            ID_p1 = _input_indices[i][k];
            ID_p2 = _input_indices[i][k+1]; // 远处点

            float x0 = incloud->points[ID_p0].x;
            float x1 = incloud->points[ID_p1].x;
            float x2 = incloud->points[ID_p2].x;

            float y0 = incloud->points[ID_p0].y;
            float y1 = incloud->points[ID_p1].y;
            float y2 = incloud->points[ID_p2].y;

            float z0 = incloud->points[ID_p0].z;
            float z1 = incloud->points[ID_p1].z;
            float z2 = incloud->points[ID_p2].z;

            // 1、平滑度特征
            // 模的平方，三点之间距离不是一样长不具有代表性
            // float Cost = sqrt(pow(x0 + x2 - 2* x1, 2)+ pow(y0 + y2 - 2* y1, 2));

            // 一条直线上三点构成的两向量之间的夹角θ(对于地面点不好去除，可以去除特别远的点)
            float Cost_up = (x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1);
            float Cost_down = sqrt( pow((x1 - x0), 2) + pow((y1 - y0), 2)) * sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
            float Cav_Cost = acos(abs(Cost_up / Cost_down)) * 180 / PI;

            // 2、线的长度特征,后一条线不能比前一条线长
            float ID_p0_distance = sqrt(pow(x0, 2)+pow(y0, 2)+pow(z0, 2));
            float ID_p1_distance = sqrt(pow(x1, 2)+pow(y1, 2)+pow(z1, 2));

            // 3、垂直距离特征，同一光束不同点之间的Z的距离
            float Z_different = abs(z0 - z1);

            // 4、长度距离特征，同一光束不同点之间的xyz的长度
            float xy01_ = sqrt( pow((x1 - x0), 2) + pow((y1 - y0), 2) ) ;
            float xy21_ = sqrt( pow((x1 - x2), 2) + pow((y1 - y2), 2) ) ;

            if(
            Cav_Cost >= _MinAnglus
            || Z_different >= _Max_Hight_Cost
            || ID_p0_distance >= ID_p1_distance
            || (xy01_+xy21_)/2 <= _Max_XY_Cost

            )
            {continue;}

            result[i].push_back(ID_p1);
        }

        // 去除点数过小的光束
        if (result[i].size() <= 3) {
            std::fill(result[i].begin(), result[i].end(), 0);
        }
    }

    return result;
};



// 在类外部定义静态成员变量并初始化
float Distinguish_Road::_pre_degrees = 0.0f;
int Distinguish_Road::_degrees_numb = 0;

/**
 * @brief 获取道路方向的角度
 *
 * @param incloud
 * @param input_indices
 * @return float
 */
float Distinguish_Road::Road_DirecTon_32c(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,std::vector<std::vector<size_t>> input_indices){
    float distance = 0;
    float ID = 0;
    float points_numb = 0;

    // 获得点云数量,从大到小排序
    std::vector<int> vector_1;
    vector_1.reserve(input_indices.size()); // 预留空间
    for(size_t i = 0;i < input_indices.size() ; ++i){
        if(input_indices[i].size()>_nScanRings){
            continue;
        }
        vector_1.push_back(input_indices[i].size());
    }


    sort(vector_1.begin(), vector_1.end(), std::greater<int>()); // 从大到小排序

    // 使用 set 去除重复元素，并且保持有序
    std::set<int, std::greater<int>> uniqueSet(vector_1.begin(), vector_1.end());
    // 只获取独一无二的前N个元素
    std::vector<int> maxN(uniqueSet.begin(), next(uniqueSet.begin(), std::min(4, static_cast<int>(uniqueSet.size()))));


    // 计算最近点和最远点之间的距离，距离最大为道路方向
    for (size_t i = 0;i < input_indices.size() ; ++i){
        // 在点云数目较多的光束计算距离
        if(input_indices[i].size() <= maxN.back()){
            continue;
        }

        float ID_BACK = input_indices[i].back();
        float ID_FRONT = input_indices[i].front();

        float x0 = incloud->points[ID_BACK].x;
        float x1 = incloud->points[ID_FRONT].x;

        float y0 = incloud->points[ID_BACK].y;
        float y1 = incloud->points[ID_FRONT].y;

        float distance_test = sqrt(pow(x0 - x1, 2)+pow(y0 - y1, 2));

        if (distance <= distance_test){
            distance = distance_test;
            ID = ID_BACK;
        }
    }

    // 如果这一帧的方向ID和上一帧所差角度过大，有理由怀疑是误判
    float degrees = atan2(incloud->points[ID].y, incloud->points[ID].x) * 180 / M_PI;



    // // 检查角度的变化是否超过了一个阈值，并且连续出现了足够多次，才发生改变，角度不要设置太小了
    if(_pre_degrees == 0){
        _pre_degrees = degrees;
    }

    if(abs(_pre_degrees - degrees) <= 15 || _degrees_numb >= 6){
        _pre_degrees = degrees;
        _degrees_numb = 0;
    }

    else{
        degrees = _pre_degrees;
        _degrees_numb++;
    }

    // 将角度映射到[0, 180)范围内
    if (degrees < 0) {
        degrees += 180.0;
    }

    // size_t groupIndex = floor(degrees / _angularRes);  // 向下取整


    float result = degrees;

    return result;

}



std::vector<size_t> Distinguish_Road::Road_DirecTon(
    std::vector<std::vector<size_t>> input_indices_l,
    std::vector<std::vector<size_t>> input_indices_r){
    float distance_l = 0;
    float distance_r = 0;
    float ID_l = 0;
    float ID_r = 0;

    for (size_t i = 0;i < input_indices_l.size() ; ++i){
        float ID_BACK = input_indices_l[i].back();
        float ID_FRONT = input_indices_l[i].front();

        float x0 = _incloud->points[ID_BACK].x;
        float x1 = _incloud->points[ID_FRONT].x;

        float y0 = _incloud->points[ID_BACK].y;
        float y1 = _incloud->points[ID_FRONT].y;

        float distance_t = sqrt(pow(x0 - x1, 2)+pow(y0 - y1, 2));

        if (distance_l <= distance_t){
            distance_l = distance_t;
            ID_l = i;

        }
    }

    for (size_t j = 0;j < input_indices_r.size() ; ++j){
        float ID_BACK = input_indices_r[j].back();
        float ID_FRONT = input_indices_r[j].front();

        float x0 = _incloud->points[ID_BACK].x;
        float x1 = _incloud->points[ID_FRONT].x;

        float y0 = _incloud->points[ID_BACK].x;
        float y1 = _incloud->points[ID_FRONT].x;

        float distance_t = sqrt(pow(x0 - x1, 2)+pow(y0 - y1, 2));

        if (distance_r <= distance_t){
            distance_r = distance_t;
            ID_r = j;

        }
    }

    std::vector<size_t> result;

    if (distance_l >= distance_r){
        result = input_indices_l[ID_l];
    }

    else
    {
        result = input_indices_l[ID_r];
    }

    return result;

}


/**
 * @brief 计算XY平面点到原点的距离
 *
 * @param p
 * @return float
 */
float Distinguish_Road::calcPointDistance(const pcl::PointXYZI &p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}


/**
 * @brief 过滤同一Ring上距离参差不齐的点
 *
 * @param points_by_ring
 */
void Distinguish_Road::Distance_on_Ring(std::vector<std::vector<size_t>> &points_by_ring){

    // 根据Ring创建std::vector
    std::vector<std::vector<size_t>> Distance_on_Ring(_nScanRings);
    for (size_t i = 0;i < _incloud->size(); ++i){
        int RingID = _incloud->points[i].intensity;
        Distance_on_Ring[RingID].push_back(i);

        // for(size_t k = 1; k < _input_indices[i].size(); k++){
        // }
    }




    for (size_t j = 0;j < _nScanRings; j++){

        if(Distance_on_Ring[j].size()<=10){
            continue;
        }

        // 这个Ring的所有点
        for(size_t z = 2;z < Distance_on_Ring[j].size()-2;z++){
            // 这个点附近的4个点
            float xy_distance =  0;

            for (int k = -2;k <= 2;k++){
                // 点与附近4个点的xy平面上的距离差值之和
                xy_distance += abs(
                            calcPointDistance(_incloud->points[Distance_on_Ring[j][z]])
                            - calcPointDistance(_incloud->points[Distance_on_Ring[j][z+k]])
                            );

            }



            if(xy_distance <= _Ring_diff){
                points_by_ring[j].push_back(Distance_on_Ring[j][z]);
            }
        }
    }
}


/**
 * @brief
 * 1.去除车顶被遮挡点
 * 2.把扫描线分为左侧和右侧扫描线
 * 3.把扫描线转换为光束
 * 4.根据角度过滤突变点


    // XY平面的角度大致相同,或者中间如果有一段坑也需要跳过

    // 根据Z值变化率大致相同

    // 有条件可以计算最后的平滑度，非必要

    // 判断是否进入坡道(Z值变化或者前后同一直线的距离)
    // 计算角度

    // 计算Z值
 *
 * @param slope_cloud
 * @param flat_cloud
 */
void Distinguish_Road::Distinguish_Proess(pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr flat_cloud,
                                          scanIndices scanindices,
                                          scanIndices scanindices_l,
                                          scanIndices scanindices_r
){
    _scan_indices = scanindices;
    _scan_indices_l = scanindices_l;
    _scan_indices_r = scanindices_r;

    slope_cloud->clear();
    flat_cloud ->clear();
    int Number_Per_Scan = 180 / _angularRes; // 扫描线一圈的点数，此处只有半圈
    std::vector<std::vector<size_t>> indices_t0;
    std::vector<std::vector<size_t>> indices_t1;
    std::vector<std::vector<size_t>> indices_t2;



    // 点数过少不统计
    for (int i = 0; i < _scan_indices.size(); i++) {
        if ((_scan_indices[i].second - _scan_indices[i].first) >= Number_Per_Scan / 10) {
        _with_out_up_car_indices.push_back(_scan_indices[i]);
        _with_out_up_car_indices_l.push_back(_scan_indices_l[i]);
        _with_out_up_car_indices_r.push_back(_scan_indices_r[i]);
        }
    }


    // 2.把扫描线转换为光束
    _line_indices.clear();
    _line_indices_l.clear();
    _line_indices_r.clear();
    _line_indices = Line_Proess(_with_out_up_car_indices);
    _line_indices_l = Line_Proess(_with_out_up_car_indices_l);
    _line_indices_r = Line_Proess_r(_with_out_up_car_indices_r);


    // 3.根据条件过滤突变点
    _Hight_indices.clear();
    indices_t1.clear();
    indices_t2.clear();
    indices_t1 = Hight_Proess(_line_indices_l);
    indices_t2 = Hight_Proess(_line_indices_r);

    std::vector<size_t> road_direction;
    road_direction = Road_DirecTon(_line_indices_l,_line_indices_r);


    // float size_mun1 = 0;
    // float size_mun2 = 0;
    // float size_mun3 = 0;
    // for (size_t i = 0; i < _line_indices.size(); ++i){
    //     if(_line_indices[i].back() == 0){
    //         continue;}
    //     size_mun1 = size_mun1 + _line_indices[i].size();
    // }

    // for (size_t j = 0; j < _line_indices_l.size(); ++j){
    //     if(_line_indices_l[j].back() == 0){
    //         continue;}
    //     size_mun2 = size_mun2 + _line_indices_l[j].size();
    // }

    // for (size_t j = 0; j < _line_indices_r.size(); ++j){
    //     if(_line_indices_r[j].back() == 0){
    //         continue;}
    //     size_mun3 = size_mun3 + _line_indices_r[j].size();
    // }

    // // cout << "_with_out_up_car_indices点云数量：" << _with_out_up_car_indices.back().second << endl ;
    // cout << "总点云数量：" << size_mun1 << endl ;
    // cout << "左右点云数量：" << size_mun2 + size_mun3<< endl ;

    // 测试 1
    for (size_t i=0;i < _incloud->size(); ++i ){
        flat_cloud->push_back(_incloud->points[i]);
    }

    // 测试 2
    // for (size_t i=0;i < indices_t1.size(); ++i ){
    //     for(const auto& ID_test :indices_t1[i]){
    //         slope_cloud->push_back(_incloud->points[ID_test]);
    //     }
    // }



    // // 测试 4

    // for(const auto& ID_test : road_direction){
    //     slope_cloud->push_back(_incloud->points[ID_test]);
    // }

}


/**
 * @brief 如果是连续斜坡，那么分上坡下坡是没有意义的，不如根据道路方向分段
 *
 * @param slope_cloud
 * @param flat_cloud
 */
void Distinguish_Road::Distinguish_Proess_32c(
    pcl::PointCloud<pcl::PointXYZI>::Ptr slope_cloud,
    float &anglus_,
    std::vector<std::vector<size_t>> &_orgin_indices
){
    slope_cloud->clear();


    // 计算最大组大小
    size_t maxGroup = _angular_ares_numb;



    // 原始点云转换为光束
    // _orgin_indices.clear();
    // _orgin_indices.resize(maxGroup);

    Line_Proess_32c(_incloud,_orgin_indices);



    // 1.根据Ring上每个点的距离过滤参差不齐的点
    _points_by_ring.clear();
    _points_by_ring.resize(_nScanRings);
    Distance_on_Ring(_points_by_ring);

    pcl::PointCloud<pcl::PointXYZI>::Ptr _points_by_ring_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i=0;i < _points_by_ring.size(); ++i ){
        for(const auto& A : _points_by_ring[i]){
            _points_by_ring_cloud->push_back(_incloud->points[A]);
        }
    }

    pcl::copyPointCloud(*_points_by_ring_cloud,*slope_cloud);

    // 2.把扫描线转换为光束
    _line_indices.clear();
    _line_indices.resize(maxGroup);
    Line_Proess_32c(_points_by_ring_cloud,_line_indices);

    // 3.根据条件过滤突变点
    std::vector<std::vector<size_t>> indices;
    indices.clear();
    indices = Hight_Proess_32c(_points_by_ring_cloud,_line_indices);


    anglus_ = Road_DirecTon_32c(_points_by_ring_cloud,indices);

    size_t groupIndex = floor(anglus_ / _angularRes);  // 向下取整

    // 测试 1
    // for (size_t i=0;i < _incloud->size(); ++i ){
    //     flat_cloud->push_back(_incloud->points[i]);
    // }

    // 测试 2
    // for (size_t i=0;i < _line_indices.size(); ++i ){
    //     for(const auto& ID_test : _line_indices[i]){
    //         slope_cloud->push_back(_points_by_ring_cloud->points[ID_test]);
    //     }
    // }




    // 测试 3
    // for(size_t i=0;i < _points_by_ring.size(); ++i ){
    //     for(size_t j=0;j < _points_by_ring[i].size(); ++j ){
    //         // cout << _points_by_ring[i][j] << endl;
    //         slope_cloud->push_back(_incloud->points[_points_by_ring[i][j]]);
    //     }
    // }



    // // 测试 4
    // for(const auto& ID_test : _orgin_indices[groupIndex]){
    //     slope_cloud->push_back(_incloud->points[ID_test]);
    // }

}

}
