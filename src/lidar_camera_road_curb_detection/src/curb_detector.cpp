#include "lidar_camera_road_curb_detection/curb_detector.hpp"
#include <ros/console.h>
// 使用OpenMP并行化
#include <omp.h>
using namespace CurbDetection;
double ssum=0.0,sum_lidar=0.0,sum_cam=0.0,sum_l_r=0.0;
int NN = 1;
int mmcc=0;
double tmax=0;
CurbDetector::CurbDetector()
{
  bool init_result = init(); // 是否初始化完毕
  ROS_ASSERT(init_result);   // 用于在程序执行时检查，为假则会生成错误并中止程序
}

CurbDetector::~CurbDetector(){}

// 初始化函数
bool CurbDetector::init()
{
  // ~ 在参数名称中的使用表示参数是相对于节点的私有命名空间的
  // ros::param::get("~lidar_in_topic",params::lidar_in_topic_);
  if(without_camera){
    // sub_cloud_ = nh_.subscribe(cloud_topic, 1, &CurbDetector::PointCloudCallback, this);
    // cout << " 此处已经注释 " << endl;
  }
  else{
    // 订阅话题，时间同步，sub获取地址
    cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic, 10);
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, camera_topic, 10);
    // 开始调用时间同步，多个订阅话题返回一个回调函数
    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(3), *cloud_sub_, *image_sub_);
    sync->registerCallback(boost::bind(&CurbDetector::PointCloudCallback_with_camera, this, _1, _2));
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/imageOutTopic", 10);
  }

  origin_pub_         = nh_.advertise<sensor_msgs::PointCloud2>("/origin_output", 10);
  test_pub_11_        = nh_.advertise<sensor_msgs::PointCloud2>("/test_cloud_11",10);
  test_pub_22_        = nh_.advertise<sensor_msgs::PointCloud2>("/test_cloud_22",10);
  road_direction_     = nh_.advertise<sensor_msgs::PointCloud2>("/road_direction",10);
  all_left_curb_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/all_left_curb", 10); // 左边缘
  all_right_curb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/all_right_curb", 10);
  left_curb_pub_      = nh_.advertise<sensor_msgs::PointCloud2>("/left_curb", 10); // 左边缘
  right_curb_pub_     = nh_.advertise<sensor_msgs::PointCloud2>("/right_curb", 10);
  Ground_point_       = nh_.advertise<sensor_msgs::PointCloud2>("/Ground_point",10);
  NonGround_point_    = nh_.advertise<sensor_msgs::PointCloud2>("/NonGround_point",10);
  feature_point_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/feature_point",10);
  pubMarker_          = nh_.advertise<visualization_msgs::Marker>("/road_direction_laser", 10);
  pubMarker_pic_      = nh_.advertise<visualization_msgs::Marker>("/road_direction_picture", 10);
  cloud_XYZI_pub_out     = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_XYZI_OUT",10);

  left_curb_pcl1_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("/left_curb_pcl1", 10);
  right_curb_pcl1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/right_curb_pcl1", 10);

  left_curb_udp_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/left_curb_udp", 10);
  right_curb_udp_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/right_curb_udp", 10);
  // 发布话题，发布点云
  return true;
}

void CurbDetector::PointCloudCallback_with_camera(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs,const sensor_msgs::ImageConstPtr& image_msg)
{

  ros::Time common_stamp = std::min(cloud_msgs->header.stamp, image_msg->header.stamp);
  // 重置状态标志（新增呢）
  {
    std::lock_guard<std::mutex> lock(mutex_);
    image_processed_ = false;
    cloud_processed_ = false;
  }

  image_in = cv_bridge::toCvShare(image_msg, "bgr8")->image;
  cv::Mat image_out;
  double time_taken1,time_taken2,time_taken3,t3,t2,t4,t5,t6,t7;
  scanIndices scanIDindices;

  std::string in_cloud_frame_id = cloud_msgs->header.frame_id;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter_XYZI(new pcl::PointCloud<pcl::PointXYZI>); // 滤波点云
  pcl::PointCloud<PointXYZIRT>::Ptr cloud_filter_XYZIRT(new pcl::PointCloud<PointXYZIRT>); // 滤波点云

  pcl::PointCloud<pcl::PointXYZI>::Ptr curb_candatate_ptr(new pcl::PointCloud<pcl::PointXYZI>); // 路缘候选点
  pcl::PointCloud<pcl::PointXYZI> cloud_road_direction; // 路缘道路方向

  pcl::PointCloud<pcl::PointXYZI>::Ptr Ground_Points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr NonGround_Points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr all_boundary_points_right(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr all_boundary_points_left(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr initial_boundary_points_right(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr initial_boundary_points_left(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr right_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr left_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr boundary_points_right(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr boundary_points_left(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> cloud_test11;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test22(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<PointXYZIRT> cloud_XYZIRT;
  pcl::PointCloud<PointXYZIRT> Ground_XYZIRT;
  pcl::PointCloud<PointXYZIRT> NonGround_XYZIRT;

  pcl::PointCloud<pcl::PointXYZI> cloud_XYZI_;
  pcl::PointCloud<pcl::PointXYZI> cloud_XYZI;
  pcl::PointCloud<pcl::PointXYZI> Ground_XYZI;
  pcl::PointCloud<pcl::PointXYZI> NonGround_XYZI;
  int dir;
  visualization_msgs::Marker line_list;


  // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  // 获取当前时刻的时间,用于计算计时


  double starrt = ros::Time::now().toSec();
  cout << "\033[1m\033[36m" << "\n" << "第 " << NN << " 帧: " << endl;

  NN++;



// 此处使用的是XYZI格式的点云，在kitti数据中使用
  pcl::fromROSMsg(*cloud_msgs, cloud_XYZI_);


  // 去除无效点,
  // #pragma omp parallel for
  for (size_t i = 0;i < cloud_XYZI_.size();i++){
    if(cloud_XYZI_.points[i].x > 0
      && cloud_XYZI_.points[i].x <= 40
      && sqrt(pow(cloud_XYZI_.points[i].x,2)+ pow(cloud_XYZI_.points[i].y,2)) >= 3.5
      ){
      // #pragma omp critical
      cloud_XYZI.push_back(cloud_XYZI_.points[i]);
    }
  }
  t6 = ros::Time::now().toSec();

  // cout << "\033[1m\033[36m" << "并行之前的时间 = " << t6-starrt << " 秒. " << endl;

  // 在process_with_picture函数中
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      t5 = ros::Time::now().toSec();

      // 计算点云laserID, 并将扫描线ID存为intensity
      Cloud_id laser_ID(laser_msg);
      pcl::PointCloud<PointXYZIRT>::Ptr CLOUD_XYZIRT(new pcl::PointCloud<PointXYZIRT>);
      laser_ID.processByOri_XYZI2XYZIRT(cloud_XYZI.makeShared(), CLOUD_XYZIRT);
      cloud_XYZIRT = *CLOUD_XYZIRT;


      if( use_pacthwork_ ){
        // 使用patchwork++对整个点云进行范围滤波
        boost::shared_ptr<PatchWorkpp<PointXYZIRT>> PatchworkppGroundSeg;
        PatchworkppGroundSeg.reset(new PatchWorkpp<PointXYZIRT>(patchwork_value));
        PatchworkppGroundSeg->estimate_ground(cloud_XYZIRT, Ground_XYZIRT, NonGround_XYZIRT, time_taken1);
        cout << "\033[1m\033[36m" << "计算patchwork消耗时间 = " << time_taken1 << " 秒. " << endl;
        cout << "\033[1m\033[36m" << "use_pacthwork_ = " << use_pacthwork_  << endl;
        PointXYZIRT2PointXYZI(Ground_XYZIRT,Ground_Points);
      }
      else{
      pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudMapper(new pcl::PointCloud<pcl::PointXYZI>);
      PointXYZIRT2PointXYZI(cloud_XYZIRT,completeCloudMapper);

      //分段ransc粗提取
      Ground_Seg ground(completeCloudMapper, ransacc);
      ground.ground_proess(Ground_Points, NonGround_Points);

      t3 = ros::Time::now().toSec();
      time_taken3 = - starrt + t3;

      // cout << "\033[1m\033[36m" << "计算 RANSAC 消耗时间 = " << time_taken3 << " 秒. " << endl;
      }


      // 根据之前计算的带ID的点云扫描线对地面点云进行分类
      // scanIndices scanIDindices;
      Cloud_id CLOUD_ID(laser_msg);
      pcl::PointCloud<pcl::PointXYZI>::Ptr Ground_Points_ID(new pcl::PointCloud<pcl::PointXYZI>);
      CLOUD_ID.processByIntensity(Ground_Points, Ground_Points_ID, scanIDindices);

      // cout << "scanIDindices点云数量：" << scanIDindices.back().second << endl ;
      // cout << "Ground_Points_ID点云数量：" << Ground_Points_ID->size() << endl ;

      // 前面所有工作为地面提取，后面有关论文创新点

      // 特征点提取
      Feature_Point feature_pp(Ground_Points_ID , scanIDindices, Points_Msg);
      feature_pp.extract_Features(featurePoints);
      t4 = ros::Time::now().toSec();
      // cout << "\033[1m\033[36m" << "每帧lidar点云数据处理消耗时间= " << t4-t5 << " 秒. " << endl;
      if(NN>2)
      {
        sum_lidar += t4-t5;
        // cout << "\033[1;32m" << "所有点云处理平均消耗时间: " <<  sum_lidar/(NN-2) << " 秒. " << endl;
      }
    }
    #pragma omp section
    {
      // 图片处理
      picture_proess picture_pp(image_in,cloud_XYZI,picMsg);
      picture_pp.picture_out(image_out,time_taken2,cloud_road_direction,dir);
      // dir=0;
      cout<< dir << endl;

      // 保存dir值到文件
      // std::ofstream outFile("dir_values.txt", std::ios::app);
      // if (outFile.is_open()) {
      //   outFile << dir << std::endl;
      //   outFile.close();
      // } else {
      //   std::cerr << "无法打开文件以保存dir值。" << std::endl;
      // }


      // cout << "\033[1m\033[36m" << "每帧计算处理图片消耗时间 = " << time_taken2 << " 秒. " << endl;{}
      if(NN>2)
      {
        sum_cam += time_taken2;
        // cout << "\033[1;32m" << "所有图像处理平均消耗时间: " <<  sum_cam/(NN-2) << " 秒. " << endl;
      }
    }
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 道路中心线与特征点处理,不添加图片的道路角度，仅仅使用峰值法来判断角度的版本
  // std::vector<visualization_msgs::Marker> line_list(2);
  // BoundaryPoints refinePoints(*featurePoints, laser_msg, bpMsg);
  // refinePoints.process_without_picture(NonGround_Points, boundary_points_right,boundary_points_left,line_list);

  // // 道路中心线
  // for (size_t i = 0; i < line_list.size(); ++i) {
  //   line_list[i].header.frame_id = in_cloud_frame_id;
  //   pubMarker_.publish(line_list[i]);
  // }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // 使用图片的道路中心线以及特征点处理
  BoundaryPoints refinePoints(*featurePoints, laser_msg, bpMsg);

  refinePoints.process_with_picture(all_boundary_points_right,all_boundary_points_left,boundary_points_right,boundary_points_left,initial_boundary_points_right,initial_boundary_points_left,right_filtered,left_filtered,line_list,dir,t7);
  if(tmax<t7)
  {
    tmax=t7;
  }
  if(NN>2)
  {
    // cout << "\033[1;32m" << "t7: " <<  t7<< " 秒. " << endl;
    // cout << "\033[1;32m" << "tmax: " <<  tmax<< " 秒. " << endl;
    sum_l_r+= t7;
    // cout << "\033[1;32m" << "所有左右道路边界提取平均消耗时间: " <<  sum_l_r/(NN-2) << " 秒. " << endl;
  }


  // 计时
  t2 = ros::Time::now().toSec();
  // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);


  setlocale(LC_ALL, "");
  // cout << "\033[1;32m" << "每帧总消耗时间: " <<  (- starrt + t2) << " 秒. " << endl;
  // if((- starrt + t2)<0.2)
  // { mmcc=mmcc+1;
  //   ssum += (- starrt + t2);
  //   cout << "\033[1;32m" << "所有点云和图像处理平均消耗时间: " <<  ssum/(mmcc) << " 秒. " << endl;
  // }
  if(NN>2)
  {
    ssum += (- starrt + t2);
    // cout << "\033[1;32m" << "所有点云和图像处理平均消耗时间: " <<  ssum/(NN-2) << " 秒. " << endl;
  }



  // ofstream dataFile;
  // // 朝TXT文档中写入数据
  // dataFile << time_used.count() << "\n" << endl;
  // // 关闭文档
  // dataFile.close();

  sensor_msgs::PointCloud2 test_cloud_11;
  pcl::toROSMsg(cloud_test11, test_cloud_11);
  test_cloud_11.header.frame_id = in_cloud_frame_id;
  cloud_test11.clear();


  sensor_msgs::PointCloud2 test_cloud_22;
  pcl::toROSMsg(*cloud_test22, test_cloud_22);
  test_cloud_22.header.frame_id = in_cloud_frame_id;
  cloud_test22->clear();

  // 特征点
  sensor_msgs::PointCloud2 feature_point_pub;
  pcl::toROSMsg(*featurePoints, feature_point_pub);
  feature_point_pub.header.frame_id = in_cloud_frame_id;
  featurePoints->clear();

  // 左特征点
  sensor_msgs::PointCloud2 Feature_point_L;
  pcl::toROSMsg(*all_boundary_points_left, Feature_point_L);
  Feature_point_L.header.frame_id = in_cloud_frame_id;
  all_boundary_points_left->clear();

  // 右特征点
  sensor_msgs::PointCloud2 Feature_point_R;
  pcl::toROSMsg(*all_boundary_points_right, Feature_point_R);
  Feature_point_R.header.frame_id = in_cloud_frame_id;
  all_boundary_points_right->clear();




  // 左边缘点
  sensor_msgs::PointCloud2 feature_point_0;
  pcl::toROSMsg(*boundary_points_left, feature_point_0);
  feature_point_0.header.frame_id = in_cloud_frame_id;
  boundary_points_left->clear();

  // 右边缘点
  sensor_msgs::PointCloud2 feature_point_1;
  pcl::toROSMsg(*boundary_points_right, feature_point_1);
  feature_point_1.header.frame_id = in_cloud_frame_id;
  boundary_points_right->clear();

  // 左初始边缘点
  sensor_msgs::PointCloud2 initial_feature_point_0;
  pcl::toROSMsg(*initial_boundary_points_left, initial_feature_point_0);
  initial_feature_point_0.header.frame_id = in_cloud_frame_id;
  initial_boundary_points_left->clear();

  // 右初始边缘点
  sensor_msgs::PointCloud2 initial_feature_point_1;
  pcl::toROSMsg(*initial_boundary_points_right, initial_feature_point_1);
  initial_feature_point_1.header.frame_id = in_cloud_frame_id;
  initial_boundary_points_right->clear();

  // 左杂乱边缘点
  sensor_msgs::PointCloud2 left_filtered_point;
  pcl::toROSMsg(*left_filtered, left_filtered_point);
  left_filtered_point.header.frame_id = in_cloud_frame_id;
  left_filtered->clear();

  // 右杂乱边缘点
  sensor_msgs::PointCloud2 right_filtered_point;
  pcl::toROSMsg(*right_filtered, right_filtered_point);
  right_filtered_point.header.frame_id = in_cloud_frame_id;
  right_filtered->clear();

  // 图像识别的道路方向
  sensor_msgs::PointCloud2 road_direction;
  // pcl::toROSMsg(cloud_road_direction, road_direction);
  pcl::toROSMsg(cloud_road_direction, road_direction);
  road_direction.header.frame_id = in_cloud_frame_id;
  cloud_road_direction.clear();
  // 范围点云
  sensor_msgs::PointCloud2 cloud_XYZI_pub;
  // pcl::toROSMsg(cloud_road_direction, road_direction);
  pcl::toROSMsg(cloud_XYZI, cloud_XYZI_pub);
  cloud_XYZI_pub.header.frame_id = in_cloud_frame_id;
  cloud_XYZI.clear();

  // 道路中心线
  line_list.header.frame_id = in_cloud_frame_id;
  pubMarker_.publish(line_list);


  test_pub_11_.publish(test_cloud_11);
  test_pub_22_.publish(test_cloud_22);
  feature_point_pub_.publish(feature_point_pub);

  all_left_curb_pub_.publish(Feature_point_L);
  all_right_curb_pub_.publish(Feature_point_R);
  left_curb_pub_.publish(feature_point_0);
  right_curb_pub_.publish(feature_point_1);
  road_direction_.publish(road_direction);

  cloud_XYZI_pub_out.publish(cloud_XYZI_pub);

  left_curb_pcl1_pub_.publish(initial_feature_point_0);
  right_curb_pcl1_pub_.publish(initial_feature_point_1);

  left_curb_udp_pub_.publish(left_filtered_point);
  right_curb_udp_pub_.publish(right_filtered_point);
  // 将OpenCV图像转换为ROS图像消息
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_out).toImageMsg();
  image_pub_.publish(msg);


  if(Is_XYZIRT){
    // 发布原始点云
    // sensor_msgs::PointCloud2 origin_point_cloud;
    // pcl::toROSMsg(cloud_XYZIRT, origin_point_cloud);
    // origin_point_cloud.header.frame_id = in_cloud_frame_id;
    // origin_pub_.publish(origin_point_cloud);
  }
  else{
    // 发布原始点云
    sensor_msgs::PointCloud2 origin_point_cloud;
    pcl::toROSMsg(cloud_XYZI, origin_point_cloud);
    origin_point_cloud.header.frame_id = in_cloud_frame_id;
    origin_pub_.publish(origin_point_cloud);
  }


  if(allow_groud_visualize){

    // setlocale(LC_ALL, "");
    // ROS_INFO_STREAM("\033[1;32m"<< "输入点云: " << choud_size << " -- Ground: "
    //                             << Ground_Points->size() <<  "/ NonGround: "
    //                             << NonGround_Points->size() << " (patchwork所用的时间: "
    //                             << time_taken1 << " sec)" << "\033[0m");

    Ground_Points->header.frame_id = in_cloud_frame_id;
    NonGround_Points->header.frame_id = in_cloud_frame_id;

    sensor_msgs::PointCloud2 Ground_point;
    pcl::toROSMsg(*Ground_Points, Ground_point);
    Ground_point_.publish(Ground_point);


    sensor_msgs::PointCloud2 NonGround_point;
    pcl::toROSMsg(*NonGround_Points, NonGround_point);
    NonGround_point_.publish(NonGround_point);
  }
}


// void CurbDetector::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msgs)
// {

//   double time_taken1;
//   int choud_size;
//   scanIndices scanIDindices;

//   std::string in_cloud_frame_id = cloud_msgs->header.frame_id;

//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter_XYZI(new pcl::PointCloud<pcl::PointXYZI>); // 滤波点云
//   pcl::PointCloud<PointXYZIRT>::Ptr cloud_filter_XYZIRT(new pcl::PointCloud<PointXYZIRT>); // 滤波点云

//   pcl::PointCloud<pcl::PointXYZI>::Ptr curb_candatate_ptr(new pcl::PointCloud<pcl::PointXYZI>); // 路缘候选点
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_road_direction(new pcl::PointCloud<pcl::PointXYZI>); // 路缘道路方向

//   pcl::PointCloud<pcl::PointXYZI>::Ptr Ground_Points(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr NonGround_Points(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr boundary_points_right(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr boundary_points_left(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<PointXYZIRT>::Ptr cloud_test11(new pcl::PointCloud<PointXYZIRT>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test22(new pcl::PointCloud<pcl::PointXYZI>);

//   pcl::PointCloud<PointXYZIRT> cloud_XYZIRT;
//   pcl::PointCloud<PointXYZIRT> Ground_XYZIRT;
//   pcl::PointCloud<PointXYZIRT> NonGround_XYZIRT;

//   pcl::PointCloud<pcl::PointXYZI> cloud_XYZI;
//   pcl::PointCloud<pcl::PointXYZI> Ground_XYZI;
//   pcl::PointCloud<pcl::PointXYZI> NonGround_XYZI;




//   std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();  // 获取当前时刻的时间,用于计算计时


//   if(Is_XYZIRT){
//     pcl::fromROSMsg(*cloud_msgs, cloud_XYZIRT);
//     choud_size = cloud_XYZIRT.size();

//     // 使用patchwork++对整个点云进行范围滤波
//     boost::shared_ptr<PatchWorkpp<PointXYZIRT>> PatchworkppGroundSeg;
//     PatchworkppGroundSeg.reset(new PatchWorkpp<PointXYZIRT>(patchwork_value));
//     PatchworkppGroundSeg->estimate_ground(cloud_XYZIRT, Ground_XYZIRT, NonGround_XYZIRT, time_taken1);

//     PointXYZIRT2PointXYZI(Ground_XYZIRT,Ground_Points);
//     PointXYZIRT2PointXYZI(NonGround_XYZIRT,NonGround_Points);
//   }
//   else{


//     pcl::fromROSMsg(*cloud_msgs, cloud_XYZI);
//     choud_size = cloud_XYZI.size();


//     // 计算点云laserID, 并将扫描线ID存为intensity
//     Cloud_id laser_ID(laser_msg);
//     pcl::PointCloud<PointXYZIRT>::Ptr CLOUD_XYZIRT(new pcl::PointCloud<PointXYZIRT>);
//     laser_ID.processByOri_XYZI2XYZIRT(cloud_XYZI.makeShared(), CLOUD_XYZIRT);
//     cloud_XYZIRT = *CLOUD_XYZIRT;

//     // 使用patchwork++对整个点云进行范围滤波
//     boost::shared_ptr<PatchWorkpp<PointXYZIRT>> PatchworkppGroundSeg;
//     PatchworkppGroundSeg.reset(new PatchWorkpp<PointXYZIRT>(patchwork_value));
//     PatchworkppGroundSeg->estimate_ground(cloud_XYZIRT, Ground_XYZIRT, NonGround_XYZIRT, time_taken1);
//     cout << "\033[1m\033[36m" << "计算patchwork消耗时间 = " << time_taken1 << " 秒. " << endl;
//     PointXYZIRT2PointXYZI(Ground_XYZIRT,Ground_Points);
//     PointXYZIRT2PointXYZI(NonGround_XYZIRT,NonGround_Points);

//     // 根据之前计算的带ID的点云扫描线对地面点云进行分类
//     Cloud_id CLOUD_ID(laser_msg);
//     scanIndices scanIDindices;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr Ground_Points_ID(new pcl::PointCloud<pcl::PointXYZI>);
//     CLOUD_ID.processByIntensity(Ground_Points, Ground_Points_ID, scanIDindices);

//     // cout << "scanIDindices点云数量：" << scanIDindices.back().second << endl ;
//     // cout << "Ground_Points_ID点云数量：" << Ground_Points_ID->size() << endl ;

//     // 特征点提取

//     Feature_Point feature_pp(Ground_Points_ID , scanIDindices, Points_Msg);
//     feature_pp.extract_Features(featurePoints);

//     /* =========================== 4.处理特征点 =========================== */
//     // 道路中心线
//     std::vector<visualization_msgs::Marker> line_list(2);
//     BoundaryPoints refinePoints(*featurePoints, laser_msg, bpMsg);
//     refinePoints.process(NonGround_Points, boundary_points_right,boundary_points_left,line_list);


//     // 道路中心线
//     for (size_t i = 0; i < line_list.size(); ++i) {
//       line_list[i].header.frame_id = in_cloud_frame_id;
//       pubMarker_.publish(line_list[i]);
//     }
//   }




//   // 计时
//   std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//   std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

//   setlocale(LC_ALL, "");
//   // ROS_INFO_STREAM("\033[1;32m"<< "计算处理程序总消耗时间: " << time_used.count()  << " 秒. "  << endl);



//   sensor_msgs::PointCloud2 test_cloud_11;
//   pcl::toROSMsg(*cloud_test11, test_cloud_11);
//   test_cloud_11.header.frame_id = in_cloud_frame_id;
//   cloud_test11->clear();


//   sensor_msgs::PointCloud2 test_cloud_22;
//   pcl::toROSMsg(*cloud_test22, test_cloud_22);
//   test_cloud_22.header.frame_id = in_cloud_frame_id;
//   cloud_test22->clear();

//   sensor_msgs::PointCloud2 feature_point_pub;
//   pcl::toROSMsg(*featurePoints, feature_point_pub);
//   feature_point_pub.header.frame_id = in_cloud_frame_id;
//   featurePoints->clear();

//   // 左边缘点
//   sensor_msgs::PointCloud2 feature_point_0;
//   pcl::toROSMsg(*boundary_points_left, feature_point_0);
//   feature_point_0.header.frame_id = in_cloud_frame_id;
//   boundary_points_left->clear();

//   // 右边缘点
//   sensor_msgs::PointCloud2 feature_point_1;
//   pcl::toROSMsg(*boundary_points_right, feature_point_1);
//   feature_point_1.header.frame_id = in_cloud_frame_id;
//   boundary_points_right->clear();


//   test_pub_11_.publish(test_cloud_11);
//   test_pub_22_.publish(test_cloud_22);
//   feature_point_pub_.publish(feature_point_pub);
//   left_curb_pub_.publish(feature_point_0);
//   right_curb_pub_.publish(feature_point_1);


//   if(Is_XYZIRT){
//     // 发布原始点云
//     sensor_msgs::PointCloud2 origin_point_cloud;
//     pcl::toROSMsg(cloud_XYZIRT, origin_point_cloud);
//     origin_point_cloud.header.frame_id = in_cloud_frame_id;
//     origin_pub_.publish(origin_point_cloud);
//   }
//   else{
//     // 发布原始点云
//     sensor_msgs::PointCloud2 origin_point_cloud;
//     pcl::toROSMsg(cloud_XYZI, origin_point_cloud);
//     origin_point_cloud.header.frame_id = in_cloud_frame_id;
//     origin_pub_.publish(origin_point_cloud);
//   }


//   if(allow_groud_visualize){

//     // setlocale(LC_ALL, "");
//     // ROS_INFO_STREAM("\033[1;32m"<< "输入点云: " << choud_size << " -- Ground: "
//     //                             << Ground_Points->size() <<  "/ NonGround: "
//     //                             << NonGround_Points->size() << " (patchwork所用的时间: "
//     //                             << time_taken1 << " sec)" << "\033[0m");

//     Ground_Points->header.frame_id = in_cloud_frame_id;
//     NonGround_Points->header.frame_id = in_cloud_frame_id;

//     sensor_msgs::PointCloud2 Ground_point;
//     pcl::toROSMsg(*Ground_Points, Ground_point);
//     Ground_point_.publish(Ground_point);


//     sensor_msgs::PointCloud2 NonGround_point;
//     pcl::toROSMsg(*NonGround_Points, NonGround_point);
//     NonGround_point_.publish(NonGround_point);
//   }





//   sensor_msgs::PointCloud2 road_direction;
//   pcl::toROSMsg(*cloud_road_direction, road_direction);
//   cloud_road_direction->clear();
//   road_direction_.publish(road_direction);

// }


void CurbDetector::PointXYZIRT2PointXYZI(
    const pcl::PointCloud<PointXYZIRT> in_cloud_xyzirt,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud)
{
  out_cloud->header = in_cloud_xyzirt.header;
  out_cloud->width = in_cloud_xyzirt.width;
  out_cloud->height = in_cloud_xyzirt.height;
  out_cloud->is_dense = in_cloud_xyzirt.is_dense; // points中的数据是否是有限的（有限为true）或者说是判断点云中的点是否包含 Inf/NaN这种值（包含为false）
  #pragma omp parallel for
  for(size_t i=0;i<in_cloud_xyzirt.points.size();i++)
  {
    pcl::PointXYZI p;
    p.x = in_cloud_xyzirt.points[i].x;
    p.y = in_cloud_xyzirt.points[i].y;
    p.z = in_cloud_xyzirt.points[i].z;
    p.intensity = in_cloud_xyzirt.points[i].ring;
    #pragma omp critical
    out_cloud->points.push_back(p);
  }

}

void CurbDetector::PointXYZIRT2PointXYZI_2(
    const pcl::PointCloud<PointXYZIRT> in_cloud_xyzirt,
    pcl::PointCloud<pcl::PointXYZI> out_cloud)
{
  out_cloud.header = in_cloud_xyzirt.header;
  out_cloud.width = in_cloud_xyzirt.width;
  out_cloud.height = in_cloud_xyzirt.height;
  out_cloud.is_dense = in_cloud_xyzirt.is_dense; // points中的数据是否是有限的（有限为true）或者说是判断点云中的点是否包含 Inf/NaN这种值（包含为false）
  for(size_t i=0;i<in_cloud_xyzirt.points.size();i++)
  {
    pcl::PointXYZI p;
    p.x = in_cloud_xyzirt.points[i].x;
    p.y = in_cloud_xyzirt.points[i].y;
    p.z = in_cloud_xyzirt.points[i].z;
    p.intensity = in_cloud_xyzirt.points[i].ring;
    out_cloud.points.push_back(p);
  }

}

void CurbDetector::PointXYZI2PointXYZIRT(
    const pcl::PointCloud<pcl::PointXYZI> in_cloud_xyzi,
    pcl::PointCloud<PointXYZIRT> out_cloud)
{
  out_cloud.header = in_cloud_xyzi.header;
  out_cloud.width = in_cloud_xyzi.width;
  out_cloud.height = in_cloud_xyzi.height;
  out_cloud.is_dense = in_cloud_xyzi.is_dense; // points中的数据是否是有限的（有限为true）或者说是判断点云中的点是否包含 Inf/NaN这种值（包含为false）
  for(size_t i=0;i<in_cloud_xyzi.points.size();i++)
  {
    PointXYZIRT p;
    p.x = in_cloud_xyzi.points[i].x;
    p.y = in_cloud_xyzi.points[i].y;
    p.z = in_cloud_xyzi.points[i].z;
    p.intensity = in_cloud_xyzi.points[i].intensity;
    p.ring = in_cloud_xyzi.points[i].intensity;
    out_cloud.points.push_back(p);
  }

}


template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}



// /**
//  * @brief condition 过滤器，过滤指定范围的点
//  *
//  * @param cloud 源点云
//  * @param minX, maxX, minY, maxY, minZ, maxZ 过滤范围XYZ
// */
pcl::PointCloud<pcl::PointXYZI>::Ptr CurbDetector::FilterCloud_condition(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
  // 创建一个指针以存储过滤后的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr condition_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  // 创建一个条件过滤器并使用 AND 运算符
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>());

  // 设置(x,y,z)范围
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (
      new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, minZ)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, maxZ)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, minX)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, maxX)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, minY)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, maxY)));


  pcl::ConditionalRemoval<pcl::PointXYZI> condRemoval;
  condRemoval.setInputCloud (cloud);
  condRemoval.setCondition (range_cond);
  // 确认是否保存点云过滤后的结构信息，如果设置为true的话，那么点不会移除，只是点的坐标全部变为NaN默认
  // condRemoval.setKeepOrganized(true);
  condRemoval.filter (*condition_filtered);

  // 返回过滤后的点云
  return condition_filtered;
};



pcl::PointCloud<PointXYZIRT>::Ptr CurbDetector::FilterCloud_condition_XYZIRT(
      const pcl::PointCloud<PointXYZIRT>::Ptr& cloud,
      float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
  // 创建一个指针以存储过滤后的点云
  pcl::PointCloud<PointXYZIRT>::Ptr condition_filtered(new pcl::PointCloud<PointXYZIRT>);

  // 创建一个条件过滤器并使用 AND 运算符
  pcl::ConditionAnd<PointXYZIRT>::Ptr range_cond(new pcl::ConditionAnd<PointXYZIRT>());

  // 设置(x,y,z)范围
  range_cond->addComparison(pcl::FieldComparison<PointXYZIRT>::ConstPtr (
      new pcl::FieldComparison<PointXYZIRT>("z", pcl::ComparisonOps::GT, minZ)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZIRT>::ConstPtr(
      new pcl::FieldComparison<PointXYZIRT>("z", pcl::ComparisonOps::LT, maxZ)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZIRT>::ConstPtr(
      new pcl::FieldComparison<PointXYZIRT>("x", pcl::ComparisonOps::GT, minX)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZIRT>::ConstPtr(
      new pcl::FieldComparison<PointXYZIRT>("x", pcl::ComparisonOps::LT, maxX)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZIRT>::ConstPtr(
      new pcl::FieldComparison<PointXYZIRT>("y", pcl::ComparisonOps::GT, minY)));
  range_cond->addComparison(pcl::FieldComparison<PointXYZIRT>::ConstPtr(
      new pcl::FieldComparison<PointXYZIRT>("y", pcl::ComparisonOps::LT, maxY)));


  pcl::ConditionalRemoval<PointXYZIRT> condRemoval;
  condRemoval.setInputCloud (cloud);
  condRemoval.setCondition (range_cond);
  // 确认是否保存点云过滤后的结构信息，如果设置为true的话，那么点不会移除，只是点的坐标全部变为NaN默认
  // condRemoval.setKeepOrganized(true);
  condRemoval.filter (*condition_filtered);

  // 返回过滤后的点云
  return condition_filtered;
};
