#include "lidar_camera_road_curb_detection/ufldv2_net.hpp"

// using namespace std;
// using namespace cv;

cv::dnn::Net net;
double sum = 0.0;

namespace CurbDetection {

Ultra_Fast_Lane_Detection_v2::Ultra_Fast_Lane_Detection_v2(std::string model_path, bool use_cuda)
{
    static bool net_initialized = false;
    if (!net_initialized) {
        if (model_path.empty()) {
            throw std::runtime_error("weight_path is empty; set it in the launch file");
        }
        ROS_INFO_STREAM("Loading lane model: " << model_path);
        net = cv::dnn::readNet(model_path);
        if (use_cuda) {
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
        } else {
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
        net_initialized = true;
    }
    dataset = (model_path.find("culane") != std::string::npos) ? "culane" : "tusimple";
	if (dataset == "culane")
	{
		num_row = 72;
		num_col = 81;
		crop_ratio = 0.6;
		inpHeight = 320;
		inpWidth = 1600;
	}
	else
	{
		num_row = 56;
		num_col = 41;
		crop_ratio = 0.8;
		inpHeight = 320;
		inpWidth = 800;
	}
	GenerateAnchor();
}


// 用于计算给定数组 v 的最大值的索引，根据指定的维度 dims
std::vector<int64_t> Ultra_Fast_Lane_Detection_v2::argmax_1(const float* v, const std::vector<int64_t>& dims)
{
	std::vector<int64_t> ret;
	ret.resize(dims[0] * dims[2] * dims[3]);
	// 并行
    #pragma omp parallel for collapse(2)
	for (int64_t i = 0; i < dims[2]; i++) {
		for (int64_t j = 0; j < dims[3]; j++) {
			int64_t offset = dims[3] * i + j;
			float max_val = 0;
			int64_t max_index = 0;
			for (int64_t k = 0; k < dims[1]; k++) {
				size_t index = k * dims[2] * dims[3] + offset;
				if (v[index] > max_val) {
					max_val = v[index];
					max_index = k;
				}
			}
			ret[offset] = max_index;
		}
	}
	return ret;
}

// 用于计算数组 v 中一系列元素的和，这些元素的索引按照一定的间隔和偏移来确定。
int64_t Ultra_Fast_Lane_Detection_v2::sum_valid(const std::vector<int64_t>& v, int64_t num, int64_t interval, int64_t offset)
{
	int64_t sum = 0;
	for (int64_t i = 0; i < num; i++) {
		sum += v[i * interval + offset];
	}
	return sum;
}

// 用于快速计算指数函数。
inline float fast_exp(float x)
{
	union {
		uint64_t i;
		float f;
	} v{};
	v.i = static_cast<int64_t>((1 << 23) * (1.4426950409 * x + 126.93490512f));
	return v.f;
}



//用于生成锚点，这些锚点用于检测车道线
void Ultra_Fast_Lane_Detection_v2::GenerateAnchor()
{
	for (int i = 0; i < num_row; i++)
	{
		if (dataset == "culane")
		{
			row_anchor.push_back(0.42 + i * (1.0 - 0.42) / (num_row - 1));
		}
		else
		{
			row_anchor.push_back((160 + i * (710 - 160) / (num_row - 1)) / 720.0);
		}
	}
	for (int i = 0; i < num_col; i++)
	{
		col_anchor.push_back(0.0 + i * (1.0 - 0.0) / (num_col - 1));
	}
}

//  用于释放内存和清理资源
Ultra_Fast_Lane_Detection_v2::~Ultra_Fast_Lane_Detection_v2()
{
	row_anchor.clear();
	col_anchor.clear();
}

// 用于对输入图像进行归一化处理
cv::Mat Ultra_Fast_Lane_Detection_v2::normalize_(cv::Mat img)
{
	int row = img.rows;
	int col = img.cols;
	std::vector<cv::Mat> bgrChannels(3);
	cv::split(img, bgrChannels);
	for (int c = 0; c < 3; c++)
	{
		bgrChannels[c].convertTo(bgrChannels[c], CV_32FC1, 1.0 / (255.0* std [c]), (0.0 - mean[c]) / std[c]);
	}
	cv::Mat m_normalized_mat;
	merge(bgrChannels, m_normalized_mat);
	cv::Rect rect(0, row - inpHeight, col, inpHeight);
	cv::Mat dstimg = m_normalized_mat(rect);
	return dstimg;

}

//  用于快速计算 Softmax 函数
float Ultra_Fast_Lane_Detection_v2::SoftMaxFast(const float* src, float* dst, int64_t length)
{
	const float alpha = *std::max_element(src, src + length);
	float denominator = 0.0f;
    #pragma omp simd
	for (int64_t i = 0; i < length; ++i) {
		dst[i] = fast_exp(src[i] - alpha);
		denominator += dst[i];
	}
	const float inv_denominator = 1.0f / denominator;
	#pragma omp simd
	for (int64_t i = 0; i < length; ++i) {
		dst[i] *= inv_denominator;
	}

	return 0;
}

//  用于在输入图像中检测车道线
void Ultra_Fast_Lane_Detection_v2::detect(cv::Mat& srcimg,std::vector<std::vector<cv::Point>>& line_chedapxian)
{
	// 清空结果但不释放内存
    // line_chedapxian.clear();

	double start1 = ros::Time::now().toSec();
	const int img_h = srcimg.rows;
	const int img_w = srcimg.cols;
	cv::Mat img;
	resize(srcimg, img, cv::Size(this->inpWidth, int(float(this->inpHeight) / this->crop_ratio)), cv::INTER_LINEAR);
	cv::Mat dstimg = this->normalize_(img);
	cv::Mat blob = cv::dnn::blobFromImage(dstimg);

	net.setInput(blob);


	double end1 = ros::Time::now().toSec();
    double time_take1 = end1 - start1; // 网络固定消耗0.03秒，占大头
	// sum += time_take1;
	// cout << "\033[1m\033[36m" << "网络预处理1= " << time_take1 << " 秒. " << endl;
	// cout << "\033[1m\033[36m" << "网络预处理1消耗总时间= " << sum << " 秒. " << endl;

	double start2 = ros::Time::now().toSec();

	std::vector<cv::Mat> outs;
	net.forward(outs, net.getUnconnectedOutLayersNames());   // 开始推理

	double end2 = ros::Time::now().toSec();
    double time_take2 = end2 - start2; // 网络固定消耗0.03秒，占大头
	// cout << "\033[1m\033[36m" << "detect网络耗时2 = " << time_take2 << " 秒. " << endl;
	////pred2coords
	const float* loc_row = (float*)outs[3].data;
	const float* loc_col = (float*)outs[2].data;
	const float* exist_row = (float*)outs[1].data;
	const float* exist_col = (float*)outs[0].data;

	std::vector<int64_t> loc_row_dims = { outs[3].size[0],outs[3].size[1],outs[3].size[2], outs[3].size[3] };
	int64_t num_grid_row = loc_row_dims[1];
	int64_t num_cls_row = loc_row_dims[2];
	int64_t num_lane_row = loc_row_dims[3];
	std::vector<int64_t> loc_col_dims = { outs[2].size[0],outs[2].size[1],outs[2].size[2], outs[2].size[3] };
	int64_t num_grid_col = loc_col_dims[1];
	int64_t num_cls_col = loc_col_dims[2];
	int64_t num_lane_col = loc_col_dims[3];

	std::vector<int64_t> exist_row_dims = { outs[1].size[0],outs[1].size[1],outs[1].size[2], outs[1].size[3] };
	std::vector<int64_t> exist_col_dims = { outs[0].size[0],outs[0].size[1],outs[0].size[2], outs[0].size[3] };

	std::vector<int64_t> max_indices_row = argmax_1(loc_row, loc_row_dims);
	std::vector<int64_t> valid_row = argmax_1(exist_row, exist_row_dims);
	std::vector<int64_t> max_indices_col = argmax_1(loc_col, loc_col_dims);
	std::vector<int64_t> valid_col = argmax_1(exist_col, exist_col_dims);

	// 最多5条线
	std::vector<std::vector<cv::Point>> line_list(4);
	for (int64_t i : { 1, 2 })
	{
		if (sum_valid(valid_row, num_cls_row, num_lane_row, i) > num_cls_row * 0.5)
		{
			for (int64_t k = 0; k < num_cls_row; k++)
			{
				int64_t index = k * num_lane_row + i;
				if (valid_row[index] != 0)
				{
					std::vector<float> pred_all_list;
					std::vector<int64_t> all_ind_list;
					for (int64_t all_ind = std::max(0, int(max_indices_row[index] - 1)); all_ind <= (std::min(num_grid_row - 1, max_indices_row[index]) + 1); all_ind++)
					{
						pred_all_list.push_back(loc_row[all_ind * num_cls_row * num_lane_row + index]);
						all_ind_list.push_back(all_ind);
					}
					std::vector<float> pred_all_list_softmax(pred_all_list.size());
					this->SoftMaxFast(pred_all_list.data(), pred_all_list_softmax.data(), pred_all_list.size());
					float out_temp = 0;
					for (int64_t l = 0; l < pred_all_list.size(); l++)
					{
						out_temp += pred_all_list_softmax[l] * all_ind_list[l];
					}
					float x = (out_temp + 0.5) / (num_grid_row - 1.0);
					float y = row_anchor[k];
					line_list[i].push_back(cv::Point(int(x*img_w), int(y*img_h)));
				}
			}
		}
	}

	for (int64_t i : {0, 3})
	{
		if (sum_valid(valid_col, num_cls_col, num_lane_col, i) > num_cls_col / 4)
		{
			for (int64_t k = 0; k < num_cls_col; k++)
			{
				int64_t index = k * num_lane_col + i;
				if (valid_col[index] != 0)
				{
					std::vector<float> pred_all_list;
					std::vector<int64_t> all_ind_list;
					for (int64_t all_ind = std::max(0, int(max_indices_col[index] - 1)); all_ind <= (std::min(num_grid_col - 1, max_indices_col[index]) + 1); all_ind++)
					{
						pred_all_list.push_back(loc_col[all_ind * num_cls_col * num_lane_col + index]);
						all_ind_list.push_back(all_ind);
					}
					std::vector<float> pred_all_list_softmax(pred_all_list.size());
					this->SoftMaxFast(pred_all_list.data(), pred_all_list_softmax.data(), pred_all_list.size());
					float out_temp = 0;
					for (int64_t l = 0; l < pred_all_list.size(); l++)
					{
						out_temp += pred_all_list_softmax[l] * all_ind_list[l];
					}
					float y = (out_temp + 0.5) / (num_grid_col - 1.0);
					float x = col_anchor[k];
					line_list[i].push_back(cv::Point(int(x*img_w), int(y*img_h)));
				}
			}
		}
	}
	cv::Mat drawimg = srcimg.clone();


	std::vector<std::pair<std::vector<cv::Point>,double>> line_list_sort;
	line_list_sort.clear();

	for (auto& line : line_list)
	{
		double sum_x = 0;
		double mean_x = 0;
		if(line.size()<10){
			continue;
		}
		for (auto& p : line)
		{

			sum_x += abs(p.x - inpWidth/2);
		}

		mean_x = sum_x / line.size();

		line_list_sort.push_back(make_pair(line,mean_x));
	}



    std::sort(line_list_sort.begin(), line_list_sort.end(),
              [](const std::pair<std::vector<cv::Point>, double>& a,
               const std::pair<std::vector<cv::Point>, double>& b) {
				// 元素升序排序
            return a.second < b.second;});




	// 以图像的正中心作为 x 轴还是使用雷达起始作为 x 轴好？选择图像先，看看效果
	// 可以选取 前方正方向作为x轴，选取x轴左右绝对值加起来最小的点除以2作为道路方向
	// 或者可以选取 前方正方向作为x轴，选取x轴左右绝对值加起来最小的两条线之间的点除以2作为道路方向
	if (line_list_sort.size() >= 2) {
		std::vector<cv::Point> line_;
		// 画出点的位置
		for (size_t t = 0; t < 2; t++)
		{

			line_ = line_list_sort[t].first;

			line_chedapxian.push_back(line_);
		}
	}
	else{
		// 识别的点数不够咋办？跳转雷达识别或者暴力选取 x轴作为道路方向
		line_chedapxian.clear();
	}

}


}
