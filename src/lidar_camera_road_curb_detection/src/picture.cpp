#define _CRT_SECURE_NO_WARNINGS
#include "lidar_camera_road_curb_detection/picture.hpp"
#include "lidar_camera_road_curb_detection/ufldv2_net.hpp"
#include <omp.h> // 添加OpenMP头文件
// kitti图像大小 1240 × 370
// 燕山大学图像大小 1280 × 720
// 输出图像大小 800 × 320
namespace CurbDetection {
picture_proess::picture_proess(const cv::Mat image,pcl::PointCloud<pcl::PointXYZI> &incloud,Picture_Msg picMsg){
    incloud_ = incloud;
    image_in_ = image;
    if(image_in_.rows >= 720){
        // 图像高度缩小到
    };
    image_t_ = cv::Mat::zeros(image_in_.size(), image_in_.type());

    gaussianblur_H_ = picMsg.GaussianBlur_heigt;
    gaussianblur_W_ = picMsg.GaussianBlur_width;
    canny_low_ = picMsg.canny_low;
    canny_up_ = picMsg.canny_up;
    threshold_thresh_ = picMsg.threshold_thresh;
    threshold_maxVal_ = picMsg.threshold_maxVal;
	weight_path_ = picMsg.weight_path;
    use_cuda_ = picMsg.use_cuda;
    chedaoxian.clear();

    if (picMsg.C_T_L.size() != 12) {
        throw std::invalid_argument("C_T_L must contain 12 values for a 3x4 projection matrix");
    }
    Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>> C_T_L_map(picMsg.C_T_L.data());
    C_T_L_ = C_T_L_map.cast<double>();
}




// void picture_proess::picture_cut(cv::Mat image_orgin,cv::Mat& image_cut){

//     cv::Rect roi(cv::Point(128,0),cv::Size(image_orgin.cols - 128,image_orgin.rows));
//     image_cut = image_orgin(roi);

//     threshold(image_orgin, image_cut, threshold_thresh_, threshold_maxVal_, 3);
// }
void picture_proess::picture_cut(cv::Mat image_orgin, cv::Mat& image_cut) {
    // 获取时间戳
    // double start = ros::Time::now().toSec();
    if (image_orgin.cols <= 128) {
        image_cut.release();
        return;
    }
    cv::Rect roi(128, 0, image_orgin.cols - 128, image_orgin.rows);
    cv::threshold(image_orgin(roi), image_cut, threshold_thresh_, threshold_maxVal_, cv::THRESH_TOZERO);
    // // 记录时间
    // double end = ros::Time::now().toSec();
    // double time_take = end - start;
    // std::cout<<"0time_take: "<<time_take<<std::endl;
}


float picture_proess::arctan_y1x(pcl::PointXYZI pxy){
    float ang = atan2(pxy.y,pxy.x)* 180 / M_PI;
    return ang;
}



void picture_proess::picture_out(cv::Mat& image_out_,double &time_take,pcl::PointCloud<pcl::PointXYZI>& road_direction,int& dir){
    dir = 0;
	if (image_in_.cols < 800 || image_in_.rows < 320) {
        ROS_WARN_THROTTLE(5.0, "Camera image must be at least 800x320 pixels");
        image_out_ = image_in_.clone();
        time_take = 0.0;
        dir = 0;
        return;
    }
    image_out_ = image_in_.clone();
    Ultra_Fast_Lane_Detection_v2 UFLDV2_net(weight_path_, use_cuda_);

	int c = image_in_.cols / 2; // 图像的中心列
	int r = image_in_.rows - 1; // 图像的中心行

    // 计算裁剪区域相对于原始图像的偏移量
    int offset_x = c - 400;
    int offset_y = r - 320;
    int num_left = 0;
    int num_right = 0;
    double sum_left = 0;
    double sum_right = 0;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> Static;   //创建滤波器对象
    std::vector<double> cal_dir(2);

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> road_process(2);
    for (size_t i = 0; i < road_process.size(); ++i) {
        road_process[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }


	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> road_ra(2);
    for (size_t i = 0; i < road_process.size(); ++i) {
        road_ra[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }


	cv::Mat image_cut;

    // 创建空白照片
    cv::Mat blank_image = cv::Mat::zeros(image_in_.size(), image_in_.type());


    image_cut = image_in_(cv::Range(r-320, r),cv::Range(c - 400,c + 400));



    double start = ros::Time::now().toSec();

	UFLDV2_net.detect(image_cut,chedaoxian);
    // 记录时间
    double end1 = ros::Time::now().toSec();
    double time_take1 = end1 - start;
    // std::cout<<"整个检测所用时间也就是1+2+其他: "<<time_take1<<std::endl;
    if(chedaoxian.size() >= 2){

        for (size_t i = 0; i < chedaoxian.size(); ++i) {
            for (size_t j = 0; j < chedaoxian[i].size(); ++j) {
                // 将裁剪区域的偏移量加到车道线点的坐标上
                chedaoxian[i][j].x += offset_x;
                chedaoxian[i][j].y += offset_y;
            }
        }


        for (auto& p : chedaoxian[0])
        {
            circle(blank_image, p, 3, cv::Scalar(0, 0, 255), -1);
        }

        for (auto& p : chedaoxian[1])
        {
            circle(blank_image, p, 3, cv::Scalar(255, 0, 0), -1);
        }


        for (auto& p : chedaoxian[0])
        {
            circle(image_in_, p, 3, cv::Scalar(0, 0, 255), -1);
        }

        for (auto& p : chedaoxian[1])
        {
            circle(image_in_, p, 3, cv::Scalar(255, 0, 0), -1);
        }


        image_out_ = image_cut.clone();


    }
    else{
        // cout << "网络识别空的！！！！！！！！！！！！！！！！" << endl;
        goto jieshu;
    }



    // Y = P_rect_XX × R0_rect × Tr_velo_to_cam × X,注意：其中 X = (x，y，z，1)^T, Y = (u，v，1)^T ：
    // pointCloud_C = C_T_L * pointCloud_L;
    // 实际的时间消耗挺小(平均耗时0.0008s)
    for(size_t i = 0; i < incloud_.size(); i++) {

        if(incloud_.points[i].x <= 0){
            continue;
        }
        // 点云坐标(x, y, z, 1)
        Eigen::Vector4d pointCloud_L(incloud_.points[i].x, incloud_.points[i].y, incloud_.points[i].z, 1.0);

        // 经过外参矩阵旋转平移变换后的坐标(u, v, w)
        Eigen::Vector3d pointCloud_C = (C_T_L_ * pointCloud_L).head<3>();

        double u = pointCloud_C[0]; // 变换后的X
        double v = pointCloud_C[1]; // 变换后的Y
        double w = pointCloud_C[2]; // 变换后的Z

        // 图像上点云的坐标(p_u,p_v)
        if (w <= 0.0) {
            continue;
        }
        float p_u = (int)(u / w);
        float p_v = (int)(v / w);

        // 条件：图像上坐标x < 0 或者  > cols-1
        if( (p_u < 0) || (p_u > blank_image.cols -1)  || (p_v < 0) || (p_v > blank_image.rows -1) ||(w < 0)){
            continue;
        }
        else{
            // 获取像素颜色
            cv::Vec3b color = blank_image.at<cv::Vec3b>(p_v, p_u);

            pcl::PointXYZI pt_dir;

            if (color == cv::Vec3b(0, 0, 255)){
                pt_dir = incloud_.points[i];
                // // 将点的坐标和颜色信息存储到相应的数据结构中
                road_process[0]->push_back(pt_dir);
            }

            if (color == cv::Vec3b(255, 0, 0))
            {
                pt_dir = incloud_.points[i];
                // // 将点的坐标和颜色信息存储到相应的数据结构中
                road_process[1]->push_back(pt_dir);
            }
        }
    }



    // 对于点云随机抽样一致，获取两条直线(平均耗时0.00045s)
        // 并行处理车道线
    #pragma omp parallel for
    for (size_t i = 0; i < road_process.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ransac = road_process[i];

        // 创建RANSAC模型
        pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud_ransac));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_line);
        ransac.setDistanceThreshold(0.01); // 设置距离阈值
        ransac.setMaxIterations(200);
        if (cloud_ransac->size() < 2 || !ransac.computeModel()) {
            continue;
        }

        // 获取拟合的模型系数
        // coef[0]、coef[1]、coef[2]表示直线上一点的x,y,z 坐标。
        // coef[3]、coef[4]、coef[5]表示直线的方向向量。
        Eigen::VectorXf coefficients;
        ransac.getModelCoefficients(coefficients);
        if (coefficients.size() < 6) {
            continue;
        }

        // 获取内点
        std::vector<int> inliers;
        ransac.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZI>(*cloud_ransac, inliers, *road_ra[i]);

        // 利用两点式计算斜率
        cal_dir[i] = std::atan2(coefficients[4],coefficients[3]) * 180.0 / M_PI;

        if(cal_dir[i] > 90){
            cal_dir[i] = cal_dir[i] - 180;
        }

        if(cal_dir[i] < -90){
            cal_dir[i] = cal_dir[i] + 180;
        }


        // std::cout << "向量(x,y):   " << coefficients[3] << coefficients[4] <<  std::endl;
        // std::cout << cal_dir[i] << std::endl;
    }



    for (int k = 0; k < road_ra.size(); ++k) {
        for (int i = 0; i < road_ra[k]->size(); ++i) {
            road_direction.push_back(road_ra[k]->points[i]);
        }
    }



    // for(size_t t = 0;t < road_direction.size();t++){
    //     if(road_direction.points[t].x > 0){
    //         num_left++;
    //         sum_left += arctan_y1x(road_direction.points[t]);
    //     }
    //     else{
    //         num_right++;
    //         sum_right += arctan_y1x(road_direction.points[t]);
    //     }
    // }

    // dir = sum_left/num_left + sum_right/num_right;

    dir = (cal_dir[0] + cal_dir[1])/2;

    // cout << "图像计算的角度：" << dir << endl;


    jieshu:
    double end = ros::Time::now().toSec();
    time_take = end - start; // 网络固定消耗0.03秒，占大头
}
}
