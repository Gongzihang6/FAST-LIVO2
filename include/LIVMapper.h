/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <sys/stat.h>
#include <vikit/camera_loader.h>

class LIVMapper
{
public:
    LIVMapper(ros::NodeHandle &nh);
    ~LIVMapper();

    // 初始化所有的 ROS 订阅者（接收传感器数据）和发布者（输出里程计、点云、图像）
    void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);

    // 实例化核心算法组件，例如 p_pre (点云处理)、p_imu (IMU处理)、voxelmap_manager (体素地图)、vio_manager (视觉跟踪)
    void initializeComponents();

    // 初始化用于记录轨迹、位姿、耗时等数据的日志文件
    void initializeFiles();

    // 系统的主循环。通常是一个独立的线程，不断检查缓冲区，一旦数据对齐，就触发后续的建图和里程计计算
    void run();

    // 重力对齐。在系统启动初期，利用静止或缓慢运动时的 IMU 数据，估计出准确的重力方向，这对 VIO/LIO 系统的初始化至关重要
    void gravityAlignment();

    // 处理第一帧数据。主要是记录初始时间戳，初始化第一帧的位姿
    void handleFirstFrame();

    //核心函数。执行紧耦合的状态估计（IESKF 迭代更新）
    void stateEstimationAndMapping();

    // 调度视觉处理模块，提取/跟踪图像特征点，构建光度/重投影残差
    void handleVIO();

    // 调度激光雷达处理模块，将当前帧点云与局部地图进行匹配，构建点面距离残差
    void handleLIO();

    // 工具函数：递归创建目录
    bool createDirectory(const std::string &path);

    // 工具函数：自动创建带时间戳的子文件夹，返回完整路径
    std::string createTimestampedDir(const std::string &base_dir);

    // 将构建好的全局或局部 3D 点云地图保存为 .pcd 文件，供离线查看
    void savePCD();

    // 结合 IMU 数据进行状态的正向传播（Predict 步骤），为 LIO 和 VIO 提供高频的先验位姿
    void processImu();

    // 非常关键的同步函数。它负责在时间轴上对齐 LiDAR 扫描包、图像帧和对应的 IMU 数据，打包成 meas 供主循环处理
    bool sync_packages(LidarMeasureGroup &meas);

    // 执行单次 IMU 积分推算
    void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);

    // 一个定时器回调，用于在没有 LiDAR/视觉更新时，单纯利用 IMU 高频发布预测的里程计 (给高频控制需求使用)
    void imu_prop_callback(const ros::TimerEvent &e);

    // 根据给定的旋转 rot 和平移 t，对整帧点云进行刚体变换
    void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);

    // 将单个点或向量从 IMU/Body 坐标系投影到 World (地图) 坐标系
    void pointBodyToWorld(const PointType &pi, PointType &po);

    // 将带有 RGB 颜色的 LiDAR 点从 LiDAR 坐标系转换到 IMU 坐标系（利用外参）
    void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
    void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);

    // LiDAR 点云数据的回调函数。分别处理标准格式（如 Velodyne/Ouster）和 Livox 专属定制格式
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);

    // IMU 数据的回调函数。将 IMU 数据压入 imu_buffer
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    // 相机图像数据的回调函数。将图像压入 img_buffer
    void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);

    void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
    void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
    void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
    void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
    void publish_odometry(const ros::Publisher &pubOdomAftMapped);
    void publish_mavros(const ros::Publisher &mavros_pose_publisher);
    void publish_path(const ros::Publisher pubPath);
    void readParameters(ros::NodeHandle &nh);
    template <typename T>
    void set_posestamp(T &out);
    template <typename T>
    void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
    template <typename T>
    Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
    cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg); // 工具函数，将 ROS 的 sensor_msgs::Image 转换为 OpenCV 的 cv::Mat 格式以便图像处理

    // C++ 互斥锁。用于保护缓冲区，防止 ROS 回调线程和处理主线程产生数据竞争
    std::mutex mtx_buffer, mtx_buffer_imu_prop;
    std::condition_variable sig_buffer; // 条件变量。当新的传感器数据到达且同步成功后，唤醒主线程

    SLAM_MODE slam_mode_;   // 运行模式（例如是 LIVO 模式，还是退化为 LIO / VIO 模式）
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;  // 全局统一体素地图，键是体素位置，值是指向体素八叉树节点的指针

    string root_dir;    // 日志保存的根目录
    string run_output_dir_; // 本次运行带时间戳的输出子目录
    string lid_topic, imu_topic, seq_name, img_topic;   // 在 ROS 中订阅的传感器话题名称
    V3D extT;   // LiDAR 到 IMU 的外参 (平移和旋转)
    M3D extR;

    int feats_down_size = 0, max_iterations = 0;    // IESKF (卡尔曼滤波) 每次优化的最大迭代次数

    double res_mean_last = 0.05;
    double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;  // IMU 陀螺仪和加速度计、以及曝光时间的测量噪声协方差（用于 EKF 的观测更新）
    double blind_rgb_points = 0.0;
    double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
    double filter_size_surf_min = 0;    // 点云降采样滤波器的网格大小
    double filter_size_pcd = 0;
    double _first_lidar_time = 0.0;
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;

    // 系统运行状态标志 (Flags & States)
    bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
    bool has_started_ = false;
    int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
    int pub_scan_num = 1;

    StatesGroup imu_propagate, latest_ekf_state;

    bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
    deque<sensor_msgs::Imu> prop_imu_buffer;
    sensor_msgs::Imu newest_imu;
    double latest_ekf_time;
    nav_msgs::Odometry imu_prop_odom;
    ros::Publisher pubImuPropOdom;
    double imu_time_offset = 0.0;
    double lidar_time_offset = 0.0;

    bool gravity_align_en = false, gravity_align_finished = false;

    bool sync_jump_flag = false;

    bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
    bool dense_map_en = false;
    int img_en = 1, imu_int_frame = 3;
    bool normal_en = true;
    bool exposure_estimate_en = false;
    double exposure_time_init = 0.0;
    bool inverse_composition_en = false;
    bool raycast_en = false;
    int lidar_en = 1;
    bool is_first_frame = false;
    int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
    double outlier_threshold;
    double plot_time;
    int frame_cnt;
    double img_time_offset = 0.0;
    deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
    deque<double> lid_header_time_buffer;
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
    deque<cv::Mat> img_buffer;
    deque<double> img_time_buffer;
    vector<pointWithVar> _pv_list;
    vector<double> extrinT;
    vector<double> extrinR;
    vector<double> cameraextrinT;
    vector<double> cameraextrinR;
    double IMG_POINT_COV;

    PointCloudXYZI::Ptr visual_sub_map;
    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr feats_down_body;
    PointCloudXYZI::Ptr feats_down_world;
    PointCloudXYZI::Ptr pcl_w_wait_pub;
    PointCloudXYZI::Ptr pcl_wait_pub;
    PointCloudXYZRGB::Ptr pcl_wait_save;
    PointCloudXYZI::Ptr pcl_wait_save_intensity;

    ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;

    V3D euler_cur;

    LidarMeasureGroup LidarMeasures;
    StatesGroup _state;
    StatesGroup state_propagat;

    nav_msgs::Path path;
    nav_msgs::Odometry odomAftMapped;
    geometry_msgs::Quaternion geoQuat;
    geometry_msgs::PoseStamped msg_body_pose;

    PreprocessPtr p_pre;
    ImuProcessPtr p_imu;
    VoxelMapManagerPtr voxelmap_manager;
    VIOManagerPtr vio_manager;

    ros::Publisher plane_pub;
    ros::Publisher voxel_pub;
    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_img;
    ros::Publisher pubLaserCloudFullRes;
    ros::Publisher pubNormal;
    ros::Publisher pubSubVisualMap;
    ros::Publisher pubLaserCloudEffect;
    ros::Publisher pubLaserCloudMap;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubPath;
    ros::Publisher pubLaserCloudDyn;
    ros::Publisher pubLaserCloudDynRmed;
    ros::Publisher pubLaserCloudDynDbg;
    image_transport::Publisher pubImage;
    ros::Publisher mavros_pose_publisher;
    ros::Timer imu_prop_timer;

    int frame_num = 0;
    double aver_time_consu = 0;
    double aver_time_icp = 0;
    double aver_time_map_inre = 0;
    bool colmap_output_en = false;
};
#endif