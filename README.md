# LiDAR-Camera Road Curb Detection / 激光雷达与单目相机融合的道路边缘检测

ROS1 道路边缘检测实验项目。节点同步接收 3D 激光雷达点云与单目图像：从点云中提取地面、几何特征和路缘候选点，利用相机车道线结果提供道路方向线索，再通过标定投影矩阵关联两种传感器的信息，输出左右路缘点云与可视化结果。

这个仓库整理自 `lidar_road_curb_detecter` 工作目录。项目聚焦于**检测流程的集成与实验**；UFLDv2 和 Patchwork++ 等基础算法有各自的原作者，见[参考与许可](#参考与许可)。

## 流程

```text
PointCloud2 ──→ 地面提取 ──→ 扫描线特征 ──→ 路缘候选点 ──→ RANSAC/可选贝叶斯细化 ──→ 左右路缘
       │                                            ↑
       └── 3×4 标定投影矩阵 ←── 单目图像 ←── UFLDv2 车道线检测 ──→ 道路方向
```

- `src/lidar_camera_road_curb_detection/src/`：点云处理、图像车道线推理、投影融合与 ROS 节点。
- `config/params_kitti.yaml`、`config/params_yanshan.yaml`：两组传感器和算法参数示例。
- `launch/kitti.launch`、`launch/yanshan.launch`：数据集与实测话题入口；都支持覆盖话题、权重路径与 CUDA 开关。

## 环境与构建

目标环境为 Ubuntu 20.04、ROS Noetic、C++17。需要 catkin、PCL、OpenCV DNN、Armadillo，以及 `jsk_recognition_msgs`、`cv_bridge`、`message_filters` 等在 `package.xml` 中声明的 ROS 依赖。`mlpack` 可选：安装后编入贝叶斯路缘细化；未安装时保留 RANSAC 初始路缘点。GPU 不是构建前提，默认使用 OpenCV DNN CPU 后端。

```bash
source /opt/ros/noetic/setup.bash
cd lidar_camera_road_curb_detection
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j2
source devel/setup.bash
```

## 模型、标定与运行

仓库不包含原工作目录中的约 368 MB ONNX 权重，也不包含 rosbag。准备与本代码的 **TuSimple / ResNet-18 / 320×800 UFLDv2 输出格式**兼容的 ONNX 模型，将其放在 `src/lidar_camera_road_curb_detection/weights/ufldv2_tusimple_res18_320x800.onnx`，或运行时指定 `weight_path`。模型来源和下载说明可参考[原始 UFLDv2 仓库](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)及其链接的 [ONNX 模型资源](https://github.com/PINTO0309/PINTO_model_zoo/tree/main/324_Ultra-Fast-Lane-Detection-v2)；请确认选用模型的结构与输出张量一致。

```bash
roslaunch lidar_camera_road_curb_detection kitti.launch rviz:=false \
  cloud_topic:=/kitti/velo/pointcloud \
  camera_topic:=/kitti/camera_color_left/image_raw \
  weight_path:=/absolute/path/to/ufldv2_tusimple_res18_320x800.onnx

# 使用燕山数据参数时：
roslaunch lidar_camera_road_curb_detection yanshan.launch rviz:=true \
  cloud_topic:=/velodyne_points camera_topic:=/zkhy_stereo/left/color
```

传入的点云须为 `sensor_msgs/PointCloud2`，图像须为 `sensor_msgs/Image`，两者需有可同步的时间戳。图像至少为 800×320；运行时需提供适合传感器的扫描线数量、角分辨率、安装高度等参数。若 OpenCV 确实带 CUDA DNN 支持，可添加 `use_cuda:=true`；普通安装保持默认 `false`。

**标定是运行前提。** `C_T_L` 为按行排列的 3×4 投影矩阵，代码将 LiDAR 点 `(x,y,z,1)` 映射为相机齐次像素 `(u,v,w)`，再取 `(u/w,v/w)`。请根据自己的相机内参与 LiDAR-相机外参生成 `K[R|t]` 并替换配置。`params_kitti.yaml` 仅给出一组示例；`params_yanshan.yaml` 保留原工作目录中的矩阵，数值看起来更接近外参，未经验证可直接用于像素投影，使用前必须重新核对。

主要输出包括 `/left_curb`、`/right_curb`（`PointCloud2`），`/road_direction`（`PointCloud2`），以及 `/imageOutTopic`（叠加图像）。各 launch 文件默认打开 RViz，可用 `rviz:=false` 关闭。

## 验证情况与限制

已在本地 ROS Noetic、OpenCV 4.2 环境完成 `catkin_make -j2` 编译，并用 `roslaunch --nodes` 检查两个入口。编译存在 PCL 旧类型的警告。由于没有公开权重和配套数据，本仓库尚未完成端到端推理、标定精度或检测指标验证；配置中的阈值仅为实验参数。`mlpack` 不存在时构建会提示贝叶斯细化停用，此时输出为 RANSAC 结果。

## 参考与许可

- [Ultra-Fast-Lane-Detection-v2](https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2)：图像车道线模型思路与权重格式，MIT。
- [Patchwork++ ROS](https://github.com/url-kaist/patchwork-plusplus-ros)：`patchworkpp.hpp` 的改编来源，GPL-3.0。
- [JSK Recognition](https://github.com/jsk-ros-pkg/jsk_recognition)：`jsk_recognition_msgs` ROS 依赖，由系统安装，未复制进仓库。

本仓库包含 GPL-3.0 来源代码，因此以 [GPL-3.0](LICENSE) 发布；详见 [第三方声明](THIRD_PARTY_NOTICES.md)。
