#include "lidar_camera_road_curb_detection/core_head.hpp"
#include "lidar_camera_road_curb_detection/curb_detector.hpp"
#include "lidar_camera_road_curb_detection/ground_seg.hpp"
#include "lidar_camera_road_curb_detection/feature.hpp"
#include "lidar_camera_road_curb_detection/cloud_id.hpp"
#include "lidar_camera_road_curb_detection/distinguish.hpp"

/*
kitti雷达相关参数:

型号：          Velodyne HDL-64E
扫描频率：       10 Hz
线数：          64 线
角度分辨率：     0.09 度（ 扫描的点数为360°/0.09°=4000 ）
距离精度：       2 厘米
数据采集速率：    约 1.3 百万点/秒
水平方向：       360 度
垂直方向：       26.8 度
扫描范围：       120 米
安装高度：       1.73 米
坐标方向：       前方为X，左侧为Y，上方为Z

*/

using namespace CurbDetection;




/*MAIN*/
int main (int argc, char** argv)
{
    /*initializing ROS*/
    ros::init (argc, argv, "lidar_camera_road_curb_detection");

    /*NodeHandle*/
    ros::NodeHandle nh;

    CurbDetector CD;

    ros::spin();

    return 0;
}
