#pragma once
#ifndef UFLDV2_NET_H
#define UFLDV2_NET_H


#define _CRT_SECURE_NO_WARNINGS


#include "lidar_camera_road_curb_detection/core_head.hpp"


namespace CurbDetection {

class Ultra_Fast_Lane_Detection_v2
{
public:

	Ultra_Fast_Lane_Detection_v2(std::string model_path, bool use_cuda = false);
	void detect(cv::Mat& cv_image,std::vector<std::vector<cv::Point>>& line_chedapxian);
	~Ultra_Fast_Lane_Detection_v2();  // 析构函数, 释放内存

	std::vector<int64_t> argmax_1(const float* v, const std::vector<int64_t>& dims);
	int64_t sum_valid(const std::vector<int64_t>& v, int64_t num, int64_t interval, int64_t offset);

private:
	cv::UMat u_resized;  // 用于GPU加速的UMat
	cv::Mat normalize_(cv::Mat img);
	float SoftMaxFast(const float* src, float* dst, int64_t length);
	int inpWidth;
	int inpHeight;
	float mean[3] = { 0.485, 0.456, 0.406 };
	float std[3] = { 0.229, 0.224, 0.225 };
	std::string dataset;
	int num_row;
	int num_col;
	std::vector<float> row_anchor;
	std::vector<float> col_anchor;
	float crop_ratio;
	void GenerateAnchor();
	// 在类定义中添加
	// 预分配内存
	cv::Mat blob;
	std::vector<cv::Mat> outs;
	std::vector<float> pred_all_list;
	std::vector<int64_t> all_ind_list;
	std::vector<float> pred_all_list_softmax;
	// Net net;
};

}
#endif
