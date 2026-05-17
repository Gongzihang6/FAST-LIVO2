/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"


/*
 * 作用：FAST-LIVO2系统主控类（LIVMapper）的构造函数。
 * 功能：完成系统运行前的各项内存分配、参数读取、核心处理模块（如LiDAR预处理、IMU处理、视觉VIO、体素地图等）的实例化，以及日志目录的初始化。
 * 实现了什么：为LiDAR-Inertial-Visual（激光-惯性-视觉）紧耦合里程计准备好所有的初始状态、数据结构和依赖组件，打通ROS接口与底层核心算法。
 * 怎么实现的：
 * 1. 使用初始化列表和 `assign` 函数对传感器间的外参变量赋初值。
 * 2. 通过 `std::shared_ptr::reset()` 为各个智能指针（如预处理模块、点云对象、VIO/VoxelMap管理器等）分配内存。
 * 3. 调用内部函数（如 `readParameters`、`loadVoxelConfig`）从ROS节点加载YAML配置文件中的参数。
 * 4. 生成带时间戳的日志文件夹，初始化文件输出流，配置系统输出路径。
 */
LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
    extrinT.assign(3, 0.0);     // IMU到LiDAR的外参
    extrinR.assign(9, 0.0);
    cameraextrinT.assign(3, 0.0);   // 相机到LiDAR的外参
    cameraextrinR.assign(9, 0.0);
    gnssExtrinT.assign(3, 0.0);
    wheelExtrinT.assign(3, 0.0);
    wheelExtrinR.assign(9, 0.0);

    p_pre.reset(new Preprocess());  // 预处理类，负责处理最原始的LiDAR数据
    p_imu.reset(new ImuProcess());  // IMU的预处理类

    readParameters(nh);
    VoxelMapConfig voxel_config;    // 声明并加载体素地图（Voxel Map）的结构配置
    loadVoxelConfig(nh, voxel_config);

    // 点云内存结构分配
    visual_sub_map.reset(new PointCloudXYZI());     // 视觉子图，用于 VIO 阶段局部视觉特征的跟踪
    feats_undistort.reset(new PointCloudXYZI());    // 存放经过 IMU 去除运动畸变后的 LiDAR 点云
    feats_down_body.reset(new PointCloudXYZI());    // 存放在 LiDAR 坐标系下（Body Frame）降采样后的点云
    feats_down_world.reset(new PointCloudXYZI());   // 存放转换到世界坐标系下（World Frame）的降采样后的点云
    pcl_w_wait_pub.reset(new PointCloudXYZI());
    pcl_wait_pub.reset(new PointCloudXYZI());
    pcl_wait_save.reset(new PointCloudXYZRGB());
    pcl_wait_save_intensity.reset(new PointCloudXYZI());
    voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
    vio_manager.reset(new VIOManager());
    root_dir = ROOT_DIR;
    run_output_dir_ = createTimestampedDir(std::string(ROOT_DIR) + "Log");  // 获取系统运行的根目录，并在 Log 文件夹下自动创建一个以当前时间戳命名的新文件夹
    ROS_INFO_STREAM("[LIVMapper] Run output directory: " << run_output_dir_);
    initializeFiles();  // 会在这个目录里打开各种 .txt 文件，用于记录轨迹（poses）、预处理数据、时间消耗等，方便科研分析和 Debug
    initializeComponents(); // 将前面第一步读取的外参等配置真正赋值给 voxelmap_manager 和 vio_manager 的内部变量
    vio_manager->setOutputDir(run_output_dir_);
    p_imu->setOutputDir(run_output_dir_);
    path.header.stamp = ros::Time::now();   // nav_msgs::Path 类型的 ROS 消息，用于在 Rviz 中画出无人机/机器人的运动轨迹
    path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

// 读取FAST-LIVO2/config目录下的yaml配置文件中的参数配置，赋值给LIVOMapper节点句柄
void LIVMapper::readParameters(ros::NodeHandle &nh)
{
    // nh.param("param_name", variable, default_value)：读取参数，读不到就用默认值
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<string>("gnss/topic", gnss_topic, "/mavros/global_position/raw/fix");
    nh.param<string>("wheel/topic", wheel_topic, "/odom");
    nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
    nh.param<int>("common/img_en", img_en, 1);
    nh.param<int>("common/lidar_en", lidar_en, 1);
    nh.param<string>("common/img_topic", img_topic, "/left_camera/image");

    nh.param<bool>("vio/normal_en", normal_en, true);
    nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
    nh.param<int>("vio/max_iterations", max_iterations, 5);
    nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
    nh.param<bool>("vio/raycast_en", raycast_en, false);
    nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
    nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
    nh.param<int>("vio/grid_size", grid_size, 5);
    nh.param<int>("vio/grid_n_height", grid_n_height, 17);
    nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
    nh.param<int>("vio/patch_size", patch_size, 8);
    nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);

    nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
    nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
    nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
    nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
    nh.param<double>("time_offset/gnss_time_offset", gnss_time_offset, 0.0);
    nh.param<double>("time_offset/wheel_time_offset", wheel_time_offset, 0.0);
    nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
    nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);

    nh.param<bool>("gnss/enable", gnss_en, false);
    nh.param<bool>("gnss/use_msg_cov", gnss_use_msg_cov, true);
    nh.param<bool>("gnss/use_z", gnss_use_z, true);
    nh.param<std::string>("gnss/update_mode", gnss_update_mode_, std::string("full"));
    nh.param<double>("gnss/default_cov_xy", gnss_default_cov_xy, 25.0);
    nh.param<double>("gnss/default_cov_z", gnss_default_cov_z, 64.0);
    nh.param<double>("gnss/gate", gnss_gate, 16.0);
    nh.param<int>("gnss/align_min_samples", gnss_align_min_samples_, 15);
    nh.param<int>("gnss/realign_interval", gnss_realign_interval_, 50);
    nh.param<int>("gnss/align_window_size", gnss_align_window_size_, 200);
    nh.param<bool>("gnss/fallback_z_only", gnss_fallback_z_only_, true);
    nh.param<vector<double>>("gnss/extrinsic_T", gnssExtrinT, vector<double>());

    nh.param<bool>("wheel/enable", wheel_en, false);
    nh.param<bool>("wheel/use_nhc", wheel_use_nhc, true);
    nh.param<bool>("wheel/use_pose_delta", wheel_use_pose_delta_, true);
    nh.param<double>("wheel/forward_cov", wheel_forward_cov, 0.5);
    nh.param<double>("wheel/lateral_cov", wheel_lateral_cov, 0.2);
    nh.param<double>("wheel/gate", wheel_gate, 9.0);
    nh.param<double>("wheel/pose_cov_pos", wheel_pose_cov_pos, 0.2);
    nh.param<double>("wheel/pose_cov_rot", wheel_pose_cov_rot, 0.05);
    nh.param<vector<double>>("wheel/extrinsic_T", wheelExtrinT, vector<double>());
    nh.param<vector<double>>("wheel/extrinsic_R", wheelExtrinR, vector<double>());

    nh.param<string>("evo/seq_name", seq_name, "01");
    nh.param<bool>("evo/pose_output_en", pose_output_en, false);
    nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
    nh.param<double>("imu/acc_cov", acc_cov, 1.0);
    nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
    nh.param<bool>("imu/imu_en", imu_en, false);
    nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
    nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);

    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<bool>("preprocess/hilti_en", hilti_en, false);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
    nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
    nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/type", pcd_save_type, 0);
    nh.param<bool>("image_save/img_save_en", img_save_en, false);
    nh.param<int>("image_save/interval", img_save_interval, 1);

    nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
    nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
    nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
    nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
    nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());
    nh.param<double>("debug/plot_time", plot_time, -10);
    nh.param<int>("debug/frame_cnt", frame_cnt, 6);

    nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
    nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
    nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
    nh.param<bool>("publish/dense_map_en", dense_map_en, false);

    p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents()
{
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    extT << VEC_FROM_ARRAY(extrinT);
    extR << MAT_FROM_ARRAY(extrinR);
    if (gnssExtrinT.size() == 3)
        gnss_pos_in_imu << VEC_FROM_ARRAY(gnssExtrinT);
    if (wheelExtrinT.size() == 3)
        wheel_pos_in_imu << VEC_FROM_ARRAY(wheelExtrinT);
    if (wheelExtrinR.size() == 9)
        wheel_rot_in_imu << MAT_FROM_ARRAY(wheelExtrinR);

    voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
    voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

    if (!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam))
        throw std::runtime_error("Camera model not correctly specified.");

    vio_manager->grid_size = grid_size;
    vio_manager->patch_size = patch_size;
    vio_manager->outlier_threshold = outlier_threshold;
    vio_manager->setImuToLidarExtrinsic(extT, extR);
    vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
    vio_manager->state = &_state;
    vio_manager->state_propagat = &state_propagat;
    vio_manager->max_iterations = max_iterations;
    vio_manager->img_point_cov = IMG_POINT_COV;
    vio_manager->normal_en = normal_en;
    vio_manager->inverse_composition_en = inverse_composition_en;
    vio_manager->raycast_en = raycast_en;
    vio_manager->grid_n_width = grid_n_width;
    vio_manager->grid_n_height = grid_n_height;
    vio_manager->patch_pyrimid_level = patch_pyrimid_level;
    vio_manager->exposure_estimate_en = exposure_estimate_en;
    vio_manager->colmap_output_en = colmap_output_en;
    vio_manager->initializeVIO();

    p_imu->set_extrinsic(extT, extR);
    p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_inv_expo_cov(inv_expo_cov);
    p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
    p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
    p_imu->set_imu_init_frame_num(imu_int_frame);

    if (!imu_en)
        p_imu->disable_imu();
    if (!gravity_est_en)
        p_imu->disable_gravity_est();
    if (!ba_bg_est_en)
        p_imu->disable_bias_est();
    if (!exposure_estimate_en)
        p_imu->disable_exposure_est();

    slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO
                                                      : ONLY_LO;
}

void LIVMapper::initializeFiles()
{
    if (run_output_dir_.empty())
    {
        run_output_dir_ = createTimestampedDir(std::string(ROOT_DIR) + "Log");
    }

    if (colmap_output_en)
    {
        createDirectory(run_output_dir_ + "/Colmap/sparse/0");
        fout_points.open(run_output_dir_ + "/Colmap/sparse/0/points3D.txt", std::ios::out);
    }
    if (pcd_save_en)
    {
        createDirectory(run_output_dir_ + "/pcd");
        fout_lidar_pos.open(run_output_dir_ + "/pcd/lidar_poses.txt", std::ios::out);
        if (!fout_lidar_pos.is_open())
            ROS_ERROR_STREAM("[LIVMapper] Failed to open: " << run_output_dir_ << "/pcd/lidar_poses.txt");
    }
    if (img_save_en)
    {
        createDirectory(run_output_dir_ + "/image");
        fout_visual_pos.open(run_output_dir_ + "/image/image_poses.txt", std::ios::out);
        if (!fout_visual_pos.is_open())
            ROS_ERROR_STREAM("[LIVMapper] Failed to open: " << run_output_dir_ << "/image/image_poses.txt");
    }
    fout_pre.open(run_output_dir_ + "/mat_pre.txt", std::ios::out);
    if (!fout_pre.is_open())
        ROS_ERROR_STREAM("[LIVMapper] Failed to open: " << run_output_dir_ << "/mat_pre.txt");
    fout_out.open(run_output_dir_ + "/mat_out.txt", std::ios::out);
    if (!fout_out.is_open())
        ROS_ERROR_STREAM("[LIVMapper] Failed to open: " << run_output_dir_ << "/mat_out.txt");
    fout_runtime_log_.open(run_output_dir_ + "/runtime_terminal.log", std::ios::out);
    if (!fout_runtime_log_.is_open())
        ROS_ERROR_STREAM("[LIVMapper] Failed to open: " << run_output_dir_ << "/runtime_terminal.log");
    RuntimeLogger::init(run_output_dir_ + "/runtime_terminal.log");
}

/*
 * 作用：初始化 FAST-LIVO2 系统的 ROS 消息订阅者（Subscribers）、发布者（Publishers）以及定时器（Timers）。
 * 功能：搭建算法核心与外界 ROS 数据交互的桥梁，负责接收传感器底层数据，并向外输出位姿、地图和调试可视化信息。
 * 实现了什么：绑定了 LiDAR、IMU 和 Camera 的输入回调函数；分配了不同作用的点云、里程计、平面特征等输出话题（Topic）；设定了专用于无人机飞控（MAVROS）的位姿接口和高频 IMU 预积分定时发布器。
 * 怎么实现的：
 * 1. 传入常规的 ros::NodeHandle 和专门处理图像的 image_transport。
 * 2. 根据雷达类型动态选择 Livox 自定义格式或标准 PointCloud2 格式的回调函数。
 * 3. 统一使用极大的接收队列（200000）以防止优化耗时导致底层数据丢包。
 * 4. 注册所有输出话题，并创建一个 250Hz (0.004s) 的定时器用于高频发布 IMU 传播状态。
 */
void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it)
{
    // 根据激光雷达型号选择相应的回调函数
    sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this) : nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
    sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);
    sub_img = nh.subscribe(img_topic, 200000, &LIVMapper::img_cbk, this);
    if (gnss_en)
        sub_gnss = nh.subscribe(gnss_topic, 200000, &LIVMapper::gnss_cbk, this);
    if (wheel_en)
        sub_wheel = nh.subscribe(wheel_topic, 200000, &LIVMapper::wheel_cbk, this);

    // 发布去畸变、配准到世界坐标系下的当前帧完整点云，Rviz中看到的
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
    // 发布视觉子图（Visual Sub-map）。FAST-LIVO2 的视觉后端会将 LiDAR 点云投影到图像平面，用于直接法（Direct Method）的光度误差对齐
    pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 100);
    // 发布在 IESKF 优化中真正起到约束作用的有效特征点
    pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    // 发布全局或增量式局部地图，用于整体建图结果的保存和展示
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);

    // 这是系统输出的最核心定位结果。/aft_mapped_to_init 包含了经过后端 LiDAR+视觉 联合优化后的高精度位姿
    // （通常在 10Hz 左右，与雷达扫描频率一致）。/path 则将历史位姿连成线，在 RViz 中画出移动轨迹。
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 10);

    // 体素地图与平面特征可视化
    plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
    voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);
    voxelmap_manager->voxel_map_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);

    // 动态环境滤除调试接口，为动态场景保留
    pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);
    pubLaserCloudDynRmed = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100);
    pubLaserCloudDynDbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100);

    // 无人机飞控集成与高频控制
    mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    pubImage = it.advertise("/rgb_img", 1);
    imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);

    // 使用 image_transport（专门用于在 ROS 中高效传输视频流的类）发布图像。它不仅用于监控原始画面，
    // 通常也会叠加上 VIO 直接法追踪到的特征点、极线或光流轨迹，供开发者直观评估当前视觉对齐的效果
    pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
    
}

/**
 * 多传感器系统时间戳的初始化与对齐
 * 在主循环或数据同步完成后的第一步被调用，用于确立整个系统的时间基准
 * 多传感器融合最怕“时空错位”。FAST-LIVO2 将 LiDAR 点云作为核心的空间观测数据，
 * 因此将接收到的第一帧有效 LiDAR 数据的扫描结束时间（或特征提取时间）作为整个系统的时间原点（零时刻）。
 */
void LIVMapper::handleFirstFrame()
{
    // 如果还没有处理过第一帧
    if (!is_first_frame)
    {
        _first_lidar_time = LidarMeasures.last_lio_update_time; // 记录第一帧 LiDAR 的有效时间戳
        // 将记录下的第一帧 LiDAR 时间戳传递给 IMU 处理模块（p_imu）
        // IMU 预积分的边界对齐：IMU 的频率极高（通常 200Hz-500Hz），
        // 在第一帧有效 LiDAR 点云到来之前，IMU 可能已经积累了大量的闲杂数据。
        // 将这个时间戳传给 IMU 模块，可以告诉它：“系统正式从这里开始运作”。
        p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
        is_first_frame = true;
        cout << "FIRST LIDAR FRAME!" << endl;
    }
}

// 把“初始的、可能倾斜的传感器坐标系”，旋转成“Z轴与重力平行的标准世界坐标系”
void LIVMapper::gravityAlignment()
{
    // 只有IMU初始化已经完成而且重力方向还没有对齐时才进行重力对齐
    if (!p_imu->imu_need_init && !gravity_align_finished)
    {
        std::cout << "Gravity Alignment Starts" << std::endl;
        V3D ez(0, 0, -1), gz(_state.gravity);

        // FromTwoVectors() 算出了一个最小旋转，能够将 gz (测量的倾斜重力) 旋转到 ez (完美的竖直重力)。
        Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
        M3D G_R_I0 = G_q_I0.toRotationMatrix();

        // 把状态中的位置、速度、重力向量都旋转到新的坐标系下。旋转后的坐标系就是一个以 IMU 初始位置为原点，Z轴与重力平行的标准世界坐标系了。
        _state.pos_end = G_R_I0 * _state.pos_end;
        _state.rot_end = G_R_I0 * _state.rot_end;
        _state.vel_end = G_R_I0 * _state.vel_end;
        _state.gravity = G_R_I0 * _state.gravity;
        gravity_align_finished = true;
        std::cout << "Gravity Alignment Finished" << std::endl;
    }
}

void LIVMapper::processImu()
{
    // double t0 = omp_get_wtime();

    p_imu->Process2(LidarMeasures, _state, feats_undistort);

    if (gravity_align_en)
        gravityAlignment();

    state_propagat = _state;
    voxelmap_manager->state_ = _state;
    voxelmap_manager->feats_undistort_ = feats_undistort;

    // double t_prop = omp_get_wtime();

    // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
    // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
    // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping()
{
    switch (LidarMeasures.lio_vio_flg)
    {
    case WHEEL:
        handleWheel();
        break;
    case GNSS:
        handleGNSS();
        break;
    case VIO:
        handleVIO();
        break;
    case LIO:
    case LO:
        handleLIO();
        break;
    }
}

void LIVMapper::applyStateCorrection(const Eigen::Matrix<double, DIM_STATE, 1> &dx)
{
    _state += dx;
    _state.cov = 0.5 * (_state.cov + _state.cov.transpose());
}

/*
 * 作用：通用的 ESIKF 顺序测量更新（Sequential Measurement Update）接口。
 * 功能：接收任意外部传感器（如 GNSS、轮速计、视觉或额外的 LiDAR 特征）的残差与雅可比矩阵，对当前卡尔曼滤波器的状态进行状态修正，并在更新前执行异常值剔除（Outlier Rejection）。
 * 实现了什么：实现了一个标准且安全的卡尔曼更新前置检验流程。它拦截了那些由于传感器故障、多径效应或动态障碍物产生的“离谱”观测数据，保护了系统状态免受破坏。
 * 怎么实现的：
 * 1. 检查输入矩阵维度是否合法。
 * 2. 计算新息协方差矩阵（Innovation Covariance, S）。
 * 3. 利用 LDLT 分解判断 S 矩阵的正定性，确保数值计算稳定。
 * 4. 计算观测残差的马氏距离（Mahalanobis Distance），并与设定的卡方检验阈值（mahalanobis_gate）对比，超过阈值则拒绝更新。
 */
bool LIVMapper::sequentialMeasurementUpdate(const Eigen::MatrixXd &H,
                                            const Eigen::VectorXd &residual,
                                            const Eigen::MatrixXd &noise,
                                            double mahalanobis_gate,
                                            const std::string &tag)
{
    if (H.rows() == 0 || H.rows() != residual.rows() || noise.rows() != noise.cols() || noise.rows() != H.rows())
        return false;

    // innovation 是卡尔曼滤波中著名的新息协方差矩阵 $S$，公式为：$S = H P H ^ T + R$ 
    const Eigen::Matrix<double, DIM_STATE, DIM_STATE> prior_cov = _state.cov;
    const Eigen::MatrixXd innovation = H * prior_cov * H.transpose() + noise;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation);
    if (ldlt.info() != Eigen::Success)
    {
        std::ostringstream oss;
        oss << "[ " << tag << " ] innovation matrix is not positive definite, skip update.\n";
        ROS_WARN_STREAM(oss.str());
        logRuntimeMessage(oss.str());
        return false;
    }
    // 计算当前观测残差的平方马氏距离,实现了数学公式：$D_m^2 = z^T S^{-1} z$ （其中 $z$ 就是 residual）
    const double mahalanobis = residual.transpose() * ldlt.solve(residual);
    if (mahalanobis_gate > 0.0 && mahalanobis > mahalanobis_gate)
    {
        std::ostringstream oss;
        oss << "[ " << tag << " ] reject measurement by gate, mahalanobis = " << mahalanobis << ", residual_norm = " << residual.norm() << "\n";
        ROS_WARN_STREAM(oss.str());
        logRuntimeMessage(oss.str());
        return false;
    }

    const Eigen::MatrixXd K = prior_cov * H.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(H.rows(), H.rows()));
    Eigen::Matrix<double, DIM_STATE, 1> dx = K * residual;
    applyStateCorrection(dx);

    const Eigen::Matrix<double, DIM_STATE, DIM_STATE> I = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity();
    const Eigen::Matrix<double, DIM_STATE, DIM_STATE> KH = K * H;
    _state.cov = (I - KH) * prior_cov * (I - KH).transpose() + K * noise * K.transpose();
    _state.cov = 0.5 * (_state.cov + _state.cov.transpose());

    state_propagat = _state;
    voxelmap_manager->state_ = _state;
    return true;
}

Eigen::Vector3d LIVMapper::llaToEnu(double latitude, double longitude, double altitude) const
{
    constexpr double kA = 6378137.0;
    constexpr double kE2 = 6.69437999014e-3;
    const double lat = latitude * M_PI / 180.0;
    const double lon = longitude * M_PI / 180.0;
    const double lat0 = gnss_origin_lla.x() * M_PI / 180.0;
    const double lon0 = gnss_origin_lla.y() * M_PI / 180.0;

    auto llaToEcef = [&](double lat_rad, double lon_rad, double alt_m) {
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_lon = std::sin(lon_rad);
        const double cos_lon = std::cos(lon_rad);
        const double N = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
        Eigen::Vector3d ecef;
        ecef.x() = (N + alt_m) * cos_lat * cos_lon;
        ecef.y() = (N + alt_m) * cos_lat * sin_lon;
        ecef.z() = (N * (1.0 - kE2) + alt_m) * sin_lat;
        return ecef;
    };

    const Eigen::Vector3d ecef = llaToEcef(lat, lon, altitude);
    const Eigen::Vector3d origin_ecef = llaToEcef(lat0, lon0, gnss_origin_lla.z());
    const Eigen::Vector3d delta = ecef - origin_ecef;

    Eigen::Matrix3d ecef_to_enu;
    ecef_to_enu << -std::sin(lon0), std::cos(lon0), 0.0,
        -std::sin(lat0) * std::cos(lon0), -std::sin(lat0) * std::sin(lon0), std::cos(lat0),
        std::cos(lat0) * std::cos(lon0), std::cos(lat0) * std::sin(lon0), std::sin(lat0);
    return ecef_to_enu * delta;
}

void LIVMapper::logRuntimeMessage(const std::string &message) const
{
    RuntimeLogger::log(message);
}

bool LIVMapper::estimateGnssAlignment(bool force_reestimate)
{
    if (static_cast<int>(gnss_align_enu_samples_.size()) < gnss_align_min_samples_)
        return gnss_align_estimated_;

    const int n = static_cast<int>(gnss_align_enu_samples_.size());
    // 检查是否有足够的运动距离来估计航向角
    const double motion_dist = (gnss_align_enu_samples_.back().head<2>() - gnss_align_enu_samples_.front().head<2>()).norm();
    if (motion_dist < 5.0 && !force_reestimate && !gnss_align_estimated_)
    {
        return false; // 等待足够的运动距离
    }

    Eigen::MatrixXd enu_xy(2, n), world_xy(2, n);
    double mean_enu_z = 0.0, mean_world_z = 0.0;
    for (int i = 0; i < n; ++i)
    {
        enu_xy.col(i) = gnss_align_enu_samples_[i].head<2>();
        world_xy.col(i) = gnss_align_world_samples_[i].head<2>();
        mean_enu_z += gnss_align_enu_samples_[i].z();
        mean_world_z += gnss_align_world_samples_[i].z();
    }
    mean_enu_z /= n;
    mean_world_z /= n;

    const Eigen::Vector2d enu_mean = enu_xy.rowwise().mean();
    const Eigen::Vector2d world_mean = world_xy.rowwise().mean();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (int i = 0; i < n; ++i)
        covariance += (enu_xy.col(i) - enu_mean) * (world_xy.col(i) - world_mean).transpose();

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R2 = svd.matrixV() * svd.matrixU().transpose();
    if (R2.determinant() < 0.0)
    {
        Eigen::Matrix2d V = svd.matrixV();
        V.col(1) *= -1.0;
        R2 = V * svd.matrixU().transpose();
    }

    gnss_align_rot_.setIdentity();
    gnss_align_rot_.block<2, 2>(0, 0) = R2;
    gnss_align_trans_.head<2>() = world_mean - R2 * enu_mean;
    gnss_align_trans_.z() = mean_world_z - mean_enu_z;
    const bool first_init = !gnss_align_estimated_;
    gnss_align_estimated_ = true;
    gnss_events_since_realign_ = 0;

    std::ostringstream oss;
    oss << "[ GNSS ] alignment " << (first_init && !force_reestimate ? "initialized" : "re-estimated")
        << " with " << n << " samples, yaw(deg)="
        << std::atan2(R2(1, 0), R2(0, 0)) * 57.295779513 << ", trans="
        << gnss_align_trans_.transpose() << "\n";
    ROS_INFO_STREAM(oss.str());
    logRuntimeMessage(oss.str());
    return true;
}

Eigen::Vector3d LIVMapper::transformGnssToWorld(const Eigen::Vector3d &enu) const
{
    return gnss_align_rot_ * enu + gnss_align_trans_;
}

bool LIVMapper::buildAuxiliaryEvent(MeasureGroup &measure, double event_time)
{
    if (imu_en && last_timestamp_imu < event_time)
        return false;

    measure.event_time = event_time;
    measure.lio_time = event_time;
    measure.vio_time = event_time;
    measure.imu.clear();

    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
        const double imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > event_time)
            break;
        if (imu_time > LidarMeasures.last_lio_update_time)
            measure.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return true;
}

bool LIVMapper::prepareWheelEvent(LidarMeasureGroup &meas, double event_time)
{
    if (!wheel_en || wheel_buffer.empty())
        return false;

    MeasureGroup m;
    if (!buildAuxiliaryEvent(m, event_time))
        return false;

    m.has_wheel = true;
    m.wheel = wheel_buffer.front();
    wheel_buffer.pop_front();
    meas.measures.clear();
    meas.measures.push_back(m);
    meas.lio_vio_flg = WHEEL;
    return true;
}

bool LIVMapper::prepareGNSSEvent(LidarMeasureGroup &meas, double event_time)
{
    if (!gnss_en || gnss_buffer.empty())
        return false;

    MeasureGroup m;
    if (!buildAuxiliaryEvent(m, event_time))
        return false;

    m.has_gnss = true;
    m.gnss = gnss_buffer.front();
    gnss_buffer.pop_front();
    meas.measures.clear();
    meas.measures.push_back(m);
    meas.lio_vio_flg = GNSS;
    return true;
}

double LIVMapper::nextWheelTime() const
{
    return (wheel_en && !wheel_buffer.empty()) ? wheel_buffer.front().timestamp : 1e18;
}

double LIVMapper::nextGNSSTime() const
{
    return (gnss_en && !gnss_buffer.empty()) ? gnss_buffer.front().timestamp : 1e18;
}

void LIVMapper::dropStaleAuxiliaryMeasurements(double reference_time)
{
    while (!wheel_buffer.empty() && wheel_buffer.front().timestamp <= reference_time + 1e-6)
        wheel_buffer.pop_front();
    while (!gnss_buffer.empty() && gnss_buffer.front().timestamp <= reference_time + 1e-6)
        gnss_buffer.pop_front();
}

void LIVMapper::handleWheel()
{
    if (LidarMeasures.measures.empty() || !LidarMeasures.measures.back().has_wheel)
        return;

    const WheelData &wheel = LidarMeasures.measures.back().wheel;
    bool updated = false;
    double residual_norm = 0.0;

    if (wheel_use_pose_delta_ && wheel.has_pose && last_wheel_state_valid_ && last_wheel_data_.has_pose)
    {
        Eigen::Isometry3d T_w_prev = Eigen::Isometry3d::Identity();
        T_w_prev.linear() = last_wheel_data_.orientation.toRotationMatrix();
        T_w_prev.translation() = last_wheel_data_.position;
        Eigen::Isometry3d T_w_curr = Eigen::Isometry3d::Identity();
        T_w_curr.linear() = wheel.orientation.toRotationMatrix();
        T_w_curr.translation() = wheel.position;
        const Eigen::Isometry3d T_w_delta = T_w_prev.inverse() * T_w_curr;

        Eigen::Isometry3d T_i_wheel = Eigen::Isometry3d::Identity();
        T_i_wheel.linear() = wheel_rot_in_imu.transpose();
        T_i_wheel.translation() = wheel_pos_in_imu;
        const Eigen::Isometry3d T_wheel_i = T_i_wheel.inverse();
        const Eigen::Isometry3d T_i_delta_meas = T_i_wheel * T_w_delta * T_wheel_i;

        const M3D R_prev = last_wheel_state_.rot_end;
        const V3D p_prev = last_wheel_state_.pos_end;
        const M3D R_pred = R_prev.transpose() * _state.rot_end;
        const V3D t_pred = R_prev.transpose() * (_state.pos_end - p_prev);

        Eigen::Matrix<double, 6, 1> residual_pose = Eigen::Matrix<double, 6, 1>::Zero();
        const M3D R_residual = R_pred.transpose() * T_i_delta_meas.rotation();
        residual_pose.block<3, 1>(0, 0) = Log(R_residual);
        residual_pose.block<3, 1>(3, 0) = T_i_delta_meas.translation() - t_pred;
        residual_norm = residual_pose.norm();

        Eigen::Matrix<double, 6, DIM_STATE> H_pose = Eigen::Matrix<double, 6, DIM_STATE>::Zero();
        H_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H_pose.block<3, 3>(3, 3) = R_prev.transpose();

        Eigen::Matrix<double, 6, 6> noise_pose = Eigen::Matrix<double, 6, 6>::Zero();
        noise_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::max(1e-4, wheel_pose_cov_rot);
        noise_pose.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::max(1e-4, wheel_pose_cov_pos);
        updated = sequentialMeasurementUpdate(H_pose, residual_pose, noise_pose, wheel_gate, "WheelPose");
    }

    if (!updated)
    {
        const V3D vel_imu = _state.rot_end.transpose() * _state.vel_end;
        V3D omega_imu = V3D::Zero();
        if (!LidarMeasures.measures.back().imu.empty())
        {
            const auto &imu_msg = LidarMeasures.measures.back().imu.back();
            omega_imu << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
            omega_imu -= _state.bias_g;
        }
        const V3D vel_at_wheel_in_imu = vel_imu + omega_imu.cross(wheel_pos_in_imu);
        const V3D vel_wheel = wheel_rot_in_imu * vel_at_wheel_in_imu;

        Eigen::Vector2d residual;
        residual << wheel.linear_velocity.x() - vel_wheel.x(), -vel_wheel.y();
        residual_norm = residual.norm();

        Eigen::Matrix<double, 2, DIM_STATE> H = Eigen::Matrix<double, 2, DIM_STATE>::Zero();
        M3D vel_at_wheel_hat;
        vel_at_wheel_hat << SKEW_SYM_MATRX(vel_at_wheel_in_imu);
        H.block<2, 3>(0, 0) = (wheel_rot_in_imu * (-vel_at_wheel_hat)).topRows<2>();
        H.block<2, 3>(0, 7) = (wheel_rot_in_imu * _state.rot_end.transpose()).topRows<2>();

        Eigen::Matrix2d noise = Eigen::Matrix2d::Zero();
        noise(0, 0) = std::max(1e-4, wheel_forward_cov);
        noise(1, 1) = std::max(1e-4, wheel_use_nhc ? wheel_lateral_cov : 1e6);
        updated = sequentialMeasurementUpdate(H, residual, noise, wheel_gate, "WheelVel");
    }

    if (updated)
    {
        wheel_accept_count_++;
        wheel_residual_accum_ += residual_norm;
        ekf_finish_once = true;
        if (imu_prop_enable)
        {
            latest_ekf_state = _state;
            latest_ekf_time = LidarMeasures.last_lio_update_time;
            state_update_flg = true;
        }
    }
    else
    {
        wheel_reject_count_++;
    }

    last_wheel_data_ = wheel;
    last_wheel_state_ = _state;
    last_wheel_state_valid_ = true;

    if ((wheel_accept_count_ + wheel_reject_count_) % 100 == 0)
    {
        std::ostringstream oss;
        const size_t total = wheel_accept_count_ + wheel_reject_count_;
        const double mean_res = wheel_accept_count_ > 0 ? wheel_residual_accum_ / wheel_accept_count_ : 0.0;
        oss << "[ Wheel ] accept=" << wheel_accept_count_ << ", reject=" << wheel_reject_count_
            << ", mean_residual=" << mean_res << "\n";
        logRuntimeMessage(oss.str());
    }
}

void LIVMapper::handleGNSS()
{
    if (LidarMeasures.measures.empty() || !LidarMeasures.measures.back().has_gnss)
        return;

    GNSSData &gnss = LidarMeasures.measures.back().gnss;
    gnss_align_enu_samples_.push_back(gnss.enu);
    gnss_align_world_samples_.push_back(_state.pos_end);
    if (static_cast<int>(gnss_align_enu_samples_.size()) > gnss_align_window_size_)
    {
        gnss_align_enu_samples_.erase(gnss_align_enu_samples_.begin());
        gnss_align_world_samples_.erase(gnss_align_world_samples_.begin());
    }

    if (!gnss_align_estimated_)
    {
        if (!estimateGnssAlignment())
            return;
    }
    else if (gnss_realign_interval_ > 0 && gnss_events_since_realign_ >= static_cast<size_t>(gnss_realign_interval_))
    {
        estimateGnssAlignment(true);
    }
    gnss_events_since_realign_++;

    gnss.aligned_pos = transformGnssToWorld(gnss.enu);
    gnss.aligned = true;
    const V3D predicted = _state.pos_end + _state.rot_end * gnss_pos_in_imu;
    const bool use_msg_cov = gnss_use_msg_cov && gnss.covariance_valid;
    const double cov_x = std::max(1e-4, use_msg_cov ? gnss.covariance.x() : gnss_default_cov_xy);
    const double cov_y = std::max(1e-4, use_msg_cov ? gnss.covariance.y() : gnss_default_cov_xy);
    const double cov_z = std::max(1e-4, use_msg_cov ? gnss.covariance.z() : gnss_default_cov_z);
    M3D lever_hat;
    lever_hat << SKEW_SYM_MATRX(gnss_pos_in_imu);
    const Eigen::MatrixXd rot_jac = -_state.rot_end * lever_hat;

    auto buildGnssMeasurement = [&](const std::string &mode,
                                    Eigen::MatrixXd &H_out,
                                    Eigen::VectorXd &residual_out,
                                    Eigen::MatrixXd &noise_out) {
        int dim = 3;
        if (mode == "xy_only")
            dim = 2;
        else if (mode == "z_only")
            dim = 1;
        else if (!gnss_use_z)
            dim = 2;

        H_out = Eigen::MatrixXd::Zero(dim, DIM_STATE);
        residual_out = Eigen::VectorXd::Zero(dim);
        noise_out = Eigen::MatrixXd::Zero(dim, dim);

        if (mode == "z_only")
        {
            residual_out(0) = gnss.aligned_pos.z() - predicted.z();
            H_out.block<1, 3>(0, 0) = rot_jac.row(2);
            H_out.block<1, 3>(0, 3) = Eigen::RowVector3d(0.0, 0.0, 1.0);
            noise_out(0, 0) = cov_z;
            return;
        }

        if (dim == 3)
            residual_out = gnss.aligned_pos - predicted;
        else
            residual_out = gnss.aligned_pos.head<2>() - predicted.head<2>();

        H_out.block(0, 0, dim, 3) = rot_jac.topRows(dim);
        H_out.block(0, 3, dim, 3) = Eigen::MatrixXd::Identity(dim, 3);

        if (dim == 2)
        {
            noise_out(0, 0) = cov_x;
            noise_out(1, 1) = cov_y;
        }
        else if (mode == "z_priority")
        {
            noise_out(0, 0) = cov_x * 25.0;
            noise_out(1, 1) = cov_y * 25.0;
            noise_out(2, 2) = cov_z;
        }
        else
        {
            noise_out(0, 0) = cov_x;
            noise_out(1, 1) = cov_y;
            noise_out(2, 2) = cov_z;
        }
    };

    Eigen::MatrixXd H;
    Eigen::VectorXd residual;
    Eigen::MatrixXd noise;
    buildGnssMeasurement(gnss_update_mode_, H, residual, noise);
    double residual_norm = residual.norm();
    bool updated = sequentialMeasurementUpdate(H, residual, noise, gnss_gate, "GNSS");

    if (!updated && gnss_fallback_z_only_ && gnss_update_mode_ != "z_only")
    {
        Eigen::MatrixXd H_z;
        Eigen::VectorXd residual_z;
        Eigen::MatrixXd noise_z;
        buildGnssMeasurement("z_only", H_z, residual_z, noise_z);
        updated = sequentialMeasurementUpdate(H_z, residual_z, noise_z, gnss_gate, "GNSS-z-only");
        if (updated)
        {
            residual_norm = residual_z.norm();
            gnss_fallback_count_++;
            std::ostringstream oss;
            oss << "[ GNSS ] fallback to z_only succeeded, residual=" << residual_norm << "\n";
            logRuntimeMessage(oss.str());
        }
    }

    if (updated)
    {
        gnss_accept_count_++;
        gnss_residual_accum_ += residual_norm;
        ekf_finish_once = true;
        if (imu_prop_enable)
        {
            latest_ekf_state = _state;
            latest_ekf_time = LidarMeasures.last_lio_update_time;
            state_update_flg = true;
        }
    }
    else
    {
        gnss_reject_count_++;
    }

    if ((gnss_accept_count_ + gnss_reject_count_) % 50 == 0)
    {
        std::ostringstream oss;
        const double mean_res = gnss_accept_count_ > 0 ? gnss_residual_accum_ / gnss_accept_count_ : 0.0;
        oss << "[ GNSS ] accept=" << gnss_accept_count_ << ", reject=" << gnss_reject_count_
            << ", fallback=" << gnss_fallback_count_ << ", mean_residual=" << mean_res
            << ", update_mode=" << gnss_update_mode_ << "\n";
        logRuntimeMessage(oss.str());
    }
}

void LIVMapper::handleVIO()
{
    euler_cur = RotMtoEuler(_state.rot_end);
    fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
             << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
             << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;

    if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr))
    {
        std::cout << "[ VIO ] No point!!!" << std::endl;
        logRuntimeMessage("[ VIO ] No point!!!\n");
        return;
    }

    std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

    if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1))
    {
        vio_manager->plot_flag = true;
    }
    else
    {
        vio_manager->plot_flag = false;
    }

    vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

    if (imu_prop_enable)
    {
        ekf_finish_once = true;
        latest_ekf_state = _state;
        latest_ekf_time = LidarMeasures.last_lio_update_time;
        state_update_flg = true;
    }

    // int size_sub_map = vio_manager->visual_sub_map_cur.size();
    // visual_sub_map->reserve(size_sub_map);
    // for (int i = 0; i < size_sub_map; i++)
    // {
    //   PointType temp_map;
    //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
    //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
    //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
    //   temp_map.intensity = 0.;
    //   visual_sub_map->push_back(temp_map);
    // }

    publish_frame_world(pubLaserCloudFullRes, vio_manager);
    publish_img_rgb(pubImage, vio_manager);

    euler_cur = RotMtoEuler(_state.rot_end);
    fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
             << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
             << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO()
{
    euler_cur = RotMtoEuler(_state.rot_end);
    fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
             << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
             << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;

    if (feats_undistort->empty() || (feats_undistort == nullptr))
    {
        std::cout << "[ LIO ]: No point!!!" << std::endl;
        logRuntimeMessage("[ LIO ]: No point!!!\n");
        return;
    }

    double t0 = omp_get_wtime();

    downSizeFilterSurf.setInputCloud(feats_undistort);
    downSizeFilterSurf.filter(*feats_down_body);

    double t_down = omp_get_wtime();

    feats_down_size = feats_down_body->points.size();
    voxelmap_manager->feats_down_body_ = feats_down_body;
    transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
    voxelmap_manager->feats_down_world_ = feats_down_world;
    voxelmap_manager->feats_down_size_ = feats_down_size;

    if (!lidar_map_inited)
    {
        lidar_map_inited = true;
        voxelmap_manager->BuildVoxelMap();
    }

    double t1 = omp_get_wtime();

    voxelmap_manager->StateEstimation(state_propagat);
    _state = voxelmap_manager->state_;
    _pv_list = voxelmap_manager->pv_list_;

    double t2 = omp_get_wtime();

    if (imu_prop_enable)
    {
        ekf_finish_once = true;
        latest_ekf_state = _state;
        latest_ekf_time = LidarMeasures.last_lio_update_time;
        state_update_flg = true;
    }

    if (pose_output_en)
    {
        static bool pos_opend = false;
        static int ocount = 0;
        std::ofstream outFile, evoFile;
        if (!pos_opend)
        {
            createDirectory(run_output_dir_ + "/result");
            evoFile.open(run_output_dir_ + "/result/" + seq_name + ".txt", std::ios::out);
            pos_opend = true;
            if (!evoFile.is_open())
                ROS_ERROR("open fail\n");
        }
        else
        {
            evoFile.open(run_output_dir_ + "/result/" + seq_name + ".txt", std::ios::app);
            if (!evoFile.is_open())
                ROS_ERROR("open fail\n");
        }
        Eigen::Matrix4d outT;
        Eigen::Quaterniond q(_state.rot_end);
        evoFile << std::fixed;
        evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    euler_cur = RotMtoEuler(_state.rot_end);
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
    publish_odometry(pubOdomAftMapped);

    double t3 = omp_get_wtime();

    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
    transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
    for (size_t i = 0; i < world_lidar->points.size(); i++)
    {
        voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
        M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
        M3D var = voxelmap_manager->body_cov_list_[i];
        var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
              (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
        voxelmap_manager->pv_list_[i].var = var;
    }
    voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
    std::cout << "[ LIO ] Update Voxel Map" << std::endl;
    logRuntimeMessage("[ LIO ] Update Voxel Map\n");
    _pv_list = voxelmap_manager->pv_list_;

    double t4 = omp_get_wtime();

    if (voxelmap_manager->config_setting_.map_sliding_en)
    {
        voxelmap_manager->mapSliding();
    }

    PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }
    *pcl_w_wait_pub = *laserCloudWorld;

    publish_frame_world(pubLaserCloudFullRes, vio_manager);
    if (pub_effect_point_en)
        publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
    if (voxelmap_manager->config_setting_.is_pub_plane_map_)
        voxelmap_manager->pubVoxelMap();
    publish_path(pubPath);
    publish_mavros(mavros_pose_publisher);

    frame_num++;
    aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

    // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
    // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
    // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
    // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
    // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
    //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
    //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

    // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
    //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
    //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
    printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
    printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
    {
        std::ostringstream oss;
        oss << "+-------------------------------------------------------------+\n";
        oss << "|                         LIO Mapping Time                    |\n";
        oss << "+-------------------------------------------------------------+\n";
        oss << "| Algorithm Stage               | Time (secs)                 |\n";
        oss << "+-------------------------------------------------------------+\n";
        oss << "| DownSample                    | " << std::fixed << std::setprecision(6) << (t_down - t0) << "                    |\n";
        oss << "| ICP                           | " << std::fixed << std::setprecision(6) << (t2 - t1) << "                    |\n";
        oss << "| updateVoxelMap                | " << std::fixed << std::setprecision(6) << (t4 - t3) << "                    |\n";
        oss << "+-------------------------------------------------------------+\n";
        oss << "| Current Total Time            | " << std::fixed << std::setprecision(6) << (t4 - t0) << "                    |\n";
        oss << "| Average Total Time            | " << std::fixed << std::setprecision(6) << aver_time_consu << "                    |\n";
        oss << "+-------------------------------------------------------------+\n";
        logRuntimeMessage(oss.str());
    }

    euler_cur = RotMtoEuler(_state.rot_end);
    fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
             << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
             << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD()
{
    if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0)
    {
        createDirectory(run_output_dir_ + "/pcd");
        std::string raw_points_dir = run_output_dir_ + "/pcd/all_raw_points.pcd";
        std::string downsampled_points_dir = run_output_dir_ + "/pcd/all_downsampled_points.pcd";
        pcl::PCDWriter pcd_writer;

        if (img_en)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
            voxel_filter.setInputCloud(pcl_wait_save);
            voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
            voxel_filter.filter(*downsampled_cloud);

            pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
            std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                      << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;

            pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
            std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir
                      << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

            if (colmap_output_en)
            {
                fout_points << "# 3D point list with one line of data per point\n";
                fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
                for (size_t i = 0; i < downsampled_cloud->size(); ++i)
                {
                    const auto &point = downsampled_cloud->points[i];
                    fout_points << i << " "
                                << std::fixed << std::setprecision(6)
                                << point.x << " " << point.y << " " << point.z << " "
                                << static_cast<int>(point.r) << " "
                                << static_cast<int>(point.g) << " "
                                << static_cast<int>(point.b) << " "
                                << 0 << std::endl;
                }
            }
        }
        else
        {
            pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
            std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                      << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
        }
    }
}

bool LIVMapper::createDirectory(const std::string &path)
{
  struct stat st;
  if (stat(path.c_str(), &st) == 0)
  {
    if (S_ISDIR(st.st_mode))
      return true;
    ROS_WARN_STREAM("[LIVMapper] " << path << " exists but is not a directory");
    return false;
  }

  std::string cmd = "mkdir -p " + path;
  int ret = system(cmd.c_str());
  if (ret != 0)
  {
    ROS_WARN_STREAM("[LIVMapper] Failed to create directory: " << path);
    return false;
  }
  return true;
}

std::string LIVMapper::createTimestampedDir(const std::string &base_dir)
{
  std::string dir = base_dir;
  if (dir.empty()) dir = std::string(ROOT_DIR) + "Log";

  size_t pos = 0;
  while ((pos = dir.find_first_of('/', pos)) != std::string::npos)
  {
    std::string sub = dir.substr(0, pos);
    if (!sub.empty()) createDirectory(sub);
    pos++;
  }
  if (!dir.empty()) createDirectory(dir);

  std::time_t now = std::time(nullptr);
  std::tm *t = std::localtime(&now);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", t);
  std::string timestamp(buf);

  std::string run_dir = dir + "/" + timestamp;
  createDirectory(run_dir);

  return run_dir;
}

void LIVMapper::run() {
    ros::Rate rate(5000);
    int idle_count = 0;
    while (ros::ok()) {
        ros::spinOnce();

        if (!sync_packages(LidarMeasures)) {
            if (has_started_)
            {
                if (imu_buffer.empty() && lid_raw_data_buffer.empty() && img_buffer.empty() &&
                    LidarMeasures.measures.empty())
                {
                    idle_count++;
                    if (idle_count > 50)
                    {
                        ROS_INFO("All data processed, shutting down...");
                        break;
                    }
                }
                else
                {
                    idle_count = 0;
                }
            }
            rate.sleep();
            continue;
        }
        has_started_ = true;
        idle_count = 0;
        handleFirstFrame();
        processImu();
        stateEstimationAndMapping();
    }
    savePCD();
    ROS_INFO("FAST-LIVO2 finished and saved PCD.");
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
    double mean_acc_norm = p_imu->IMU_mean_acc_norm;
    acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
    angvel_avr -= imu_prop_state.bias_g;

    M3D Exp_f = Exp(angvel_avr, dt);
    /* propogation of IMU attitude */
    imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

    /* Specific acceleration (global frame) of IMU */
    V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

    /* propogation of IMU */
    imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU */
    imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
    if (p_imu->imu_need_init || !new_imu || !ekf_finish_once)
    {
        return;
    }
    mtx_buffer_imu_prop.lock();
    new_imu = false; // 控制propagate频率和IMU频率一致
    if (imu_prop_enable && !prop_imu_buffer.empty())
    {
        static double last_t_from_lidar_end_time = 0;
        if (state_update_flg)
        {
            imu_propagate = latest_ekf_state;
            // drop all useless imu pkg
            while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time))
            {
                prop_imu_buffer.pop_front();
            }
            last_t_from_lidar_end_time = 0;
            for (int i = 0; i < prop_imu_buffer.size(); i++)
            {
                double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
                double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
                // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
                V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
                V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
                prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
                last_t_from_lidar_end_time = t_from_lidar_end_time;
            }
            state_update_flg = false;
        }
        else
        {
            V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
            V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
            double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
            double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
            prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
            last_t_from_lidar_end_time = t_from_lidar_end_time;
        }

        V3D posi, vel_i;
        Eigen::Quaterniond q;
        posi = imu_propagate.pos_end;
        vel_i = imu_propagate.vel_end;
        q = Eigen::Quaterniond(imu_propagate.rot_end);
        imu_prop_odom.header.frame_id = "world";
        imu_prop_odom.header.stamp = newest_imu.header.stamp;
        imu_prop_odom.pose.pose.position.x = posi.x();
        imu_prop_odom.pose.pose.position.y = posi.y();
        imu_prop_odom.pose.pose.position.z = posi.z();
        imu_prop_odom.pose.pose.orientation.w = q.w();
        imu_prop_odom.pose.pose.orientation.x = q.x();
        imu_prop_odom.pose.pose.orientation.y = q.y();
        imu_prop_odom.pose.pose.orientation.z = q.z();
        imu_prop_odom.twist.twist.linear.x = vel_i.x();
        imu_prop_odom.twist.twist.linear.y = vel_i.y();
        imu_prop_odom.twist.twist.linear.z = vel_i.z();
        pubImuPropOdom.publish(imu_prop_odom);
    }
    mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
    PointCloudXYZI().swap(*trans_cloud);
    trans_cloud->reserve(input_cloud->size());
    for (size_t i = 0; i < input_cloud->size(); i++)
    {
        pcl::PointXYZINormal p_c = input_cloud->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        p = (rot * (extR * p + extT) + t);
        PointType pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back(pi);
    }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
    V3D p_body(pi.x, pi.y, pi.z);
    V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
    po.x = p_global(0);
    po.y = p_global(1);
    po.z = p_global(2);
    po.intensity = pi.intensity;
}

template <typename T>
void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

template <typename T>
Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
    V3D p(pi[0], pi[1], pi[2]);
    p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
    Matrix<T, 3, 1> po(p[0], p[1], p[2]);
    return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(extR * p_body_lidar + extT);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (!lidar_en)
        return;
    mtx_buffer.lock();

    double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
    // cout<<"got feature"<<endl;
    if (cur_head_time < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lid_raw_data_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lid_raw_data_buffer.push_back(ptr);
    lid_header_time_buffer.push_back(cur_head_time);
    last_timestamp_lidar = cur_head_time;

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
    if (!lidar_en)
        return;
    mtx_buffer.lock();
    livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
    // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
    // {
    //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
    //   sync_jump_flag = true;
    //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
    // }
    if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
    {
        double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
        printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
        // imu_time_offset = timediff_imu_wrt_lidar;
    }

    double cur_head_time = msg->header.stamp.toSec();
    ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
    if (cur_head_time < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lid_raw_data_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    if (!ptr || ptr->empty())
    {
        ROS_ERROR("Received an empty point cloud");
        mtx_buffer.unlock();
        return;
    }

    lid_raw_data_buffer.push_back(ptr);
    lid_header_time_buffer.push_back(cur_head_time);
    last_timestamp_lidar = cur_head_time;

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    if (!imu_en)
        return;

    if (last_timestamp_lidar < 0.0)
        return;
    // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
    double timestamp = msg->header.stamp.toSec();

    if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
    {
        ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
    }

    if (ros_driver_fix_en)
        timestamp += std::round(last_timestamp_lidar - timestamp);
    msg->header.stamp = ros::Time().fromSec(timestamp);

    mtx_buffer.lock();

    if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
        return;
    }

    // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
    // {

    //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
    //   mtx_buffer.unlock();
    //   sig_buffer.notify_all();
    //   return;
    // }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
    mtx_buffer.unlock();
    if (imu_prop_enable)
    {
        mtx_buffer_imu_prop.lock();
        if (imu_prop_enable && !p_imu->imu_need_init)
        {
            prop_imu_buffer.push_back(*msg);
        }
        newest_imu = *msg;
        new_imu = true;
        mtx_buffer_imu_prop.unlock();
    }
    sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv::Mat img;
    img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    return img;
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
    if (!img_en)
        return;
    sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));
    // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
    // {
    //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
    //   sync_jump_flag = true;
    //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
    // }

    // Hiliti2022 40Hz
    if (hilti_en)
    {
        static int frame_counter = 0;
        if (++frame_counter % 4 != 0)
            return;
    }
    // double msg_header_time =  msg->header.stamp.toSec();
    double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
    if (abs(msg_header_time - last_timestamp_img) < 0.001)
        return;
    ROS_INFO("Get image, its header time: %.6f", msg_header_time);
    if (last_timestamp_lidar < 0)
        return;

    if (msg_header_time < last_timestamp_img)
    {
        ROS_ERROR("image loop back. \n");
        return;
    }

    mtx_buffer.lock();

    double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

    if (img_time_correct - last_timestamp_img < 0.02)
    {
        ROS_WARN("Image need Jumps: %.6f", img_time_correct);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    cv::Mat img_cur = getImageFromMsg(msg);
    img_buffer.push_back(img_cur);
    img_time_buffer.push_back(img_time_correct);

    // ROS_INFO("Correct Image time: %.6f", img_time_correct);

    last_timestamp_img = img_time_correct;
    // cv::imshow("img", img);
    // cv::waitKey(1);
    // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void LIVMapper::gnss_cbk(const sensor_msgs::NavSatFix::ConstPtr &msg_in)
{
    if (!gnss_en)
        return;

    if (!std::isfinite(msg_in->latitude) || !std::isfinite(msg_in->longitude) || !std::isfinite(msg_in->altitude))
        return;

    GNSSData data;
    data.timestamp = msg_in->header.stamp.toSec() - gnss_time_offset;
    data.latitude = msg_in->latitude;
    data.longitude = msg_in->longitude;
    data.altitude = msg_in->altitude;
    data.covariance_valid = msg_in->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    if (data.covariance_valid)
    {
        data.covariance.x() = std::max(1e-4, msg_in->position_covariance[0]);
        data.covariance.y() = std::max(1e-4, msg_in->position_covariance[4]);
        data.covariance.z() = std::max(1e-4, msg_in->position_covariance[8]);
    }

    if (!gnss_origin_inited)
    {
        gnss_origin_lla << data.latitude, data.longitude, data.altitude;
        gnss_origin_inited = true;
        ROS_INFO_STREAM("[ GNSS ] ENU origin initialized at LLA = " << gnss_origin_lla.transpose());
    }

    data.enu = llaToEnu(data.latitude, data.longitude, data.altitude);

    mtx_buffer.lock();
    if (last_timestamp_gnss > 0.0 && data.timestamp < last_timestamp_gnss)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        ROS_ERROR("[ GNSS ] loop back.");
        return;
    }
    last_timestamp_gnss = data.timestamp;
    gnss_buffer.push_back(data);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void LIVMapper::wheel_cbk(const nav_msgs::Odometry::ConstPtr &msg_in)
{
    if (!wheel_en)
        return;

    WheelData data;
    data.timestamp = msg_in->header.stamp.toSec() - wheel_time_offset;
    data.linear_velocity << msg_in->twist.twist.linear.x, msg_in->twist.twist.linear.y, msg_in->twist.twist.linear.z;
    data.angular_velocity << msg_in->twist.twist.angular.x, msg_in->twist.twist.angular.y, msg_in->twist.twist.angular.z;
    data.position << msg_in->pose.pose.position.x, msg_in->pose.pose.position.y, msg_in->pose.pose.position.z;
    data.orientation = Eigen::Quaterniond(msg_in->pose.pose.orientation.w,
                                          msg_in->pose.pose.orientation.x,
                                          msg_in->pose.pose.orientation.y,
                                          msg_in->pose.pose.orientation.z);
    if (data.orientation.norm() > 1e-6)
    {
        data.orientation.normalize();
        data.has_pose = true;
    }

    mtx_buffer.lock();
    if (last_timestamp_wheel > 0.0 && data.timestamp < last_timestamp_wheel)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        ROS_ERROR("[ Wheel ] loop back.");
        return;
    }
    last_timestamp_wheel = data.timestamp;
    wheel_buffer.push_back(data);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
    if (lid_raw_data_buffer.empty() && lidar_en)
        return false;
    if (imu_buffer.empty() && imu_en)
        return false;

    switch (slam_mode_)
    {
    case ONLY_LIO:
    {
        if (meas.last_lio_update_time < 0.0)
            meas.last_lio_update_time = lid_header_time_buffer.front();
        if (!lidar_pushed)
        {
            meas.lidar = lid_raw_data_buffer.front();
            if (meas.lidar->points.size() <= 1)
                return false;

            meas.lidar_frame_beg_time = lid_header_time_buffer.front();
            meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
            meas.pcl_proc_cur = meas.lidar;
            lidar_pushed = true;
        }

        dropStaleAuxiliaryMeasurements(meas.last_lio_update_time);
        if (is_first_frame)
        {
            const double next_aux_time = std::min(nextWheelTime(), nextGNSSTime());
            if (next_aux_time + 1e-6 < meas.lidar_frame_end_time)
            {
                if (nextWheelTime() <= nextGNSSTime())
                    return prepareWheelEvent(meas, next_aux_time);
                return prepareGNSSEvent(meas, next_aux_time);
            }
        }

        if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
            return false;

        MeasureGroup m;
        m.event_time = meas.lidar_frame_end_time;
        m.lio_time = meas.lidar_frame_end_time;
        m.imu.clear();
        mtx_buffer.lock();
        while (!imu_buffer.empty())
        {
            if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time)
                break;
            if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time)
                m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();

        meas.measures.clear();
        meas.measures.push_back(m);
        meas.lio_vio_flg = LIO;
        lidar_pushed = false;
        return true;
    }

    case LIVO:
    {
        if (meas.last_lio_update_time < 0.0)
            meas.last_lio_update_time = lid_header_time_buffer.front();

        if (meas.lio_vio_flg == LIO)
        {
            if (img_buffer.empty())
                return false;
            const double img_capture_time = img_time_buffer.front() + exposure_time_init;
            meas.lio_vio_flg = VIO;
            meas.measures.clear();
            MeasureGroup m;
            m.event_time = img_capture_time;
            m.vio_time = img_capture_time;
            m.lio_time = meas.last_lio_update_time;
            m.img = img_buffer.front();
            mtx_buffer.lock();
            img_buffer.pop_front();
            img_time_buffer.pop_front();
            mtx_buffer.unlock();
            sig_buffer.notify_all();
            meas.measures.push_back(m);
            lidar_pushed = false;
            return true;
        }

        if (img_buffer.empty())
        {
            if (!is_first_frame)
                return false;

            dropStaleAuxiliaryMeasurements(meas.last_lio_update_time);
            const double next_aux_time = std::min(nextWheelTime(), nextGNSSTime());
            if (next_aux_time >= 1e18)
                return false;
            if (nextWheelTime() <= nextGNSSTime())
                return prepareWheelEvent(meas, next_aux_time);
            return prepareGNSSEvent(meas, next_aux_time);
        }

        double img_capture_time = img_time_buffer.front() + exposure_time_init;
        const double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
        const double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

        if (img_capture_time < meas.last_lio_update_time + 1e-5)
        {
            img_buffer.pop_front();
            img_time_buffer.pop_front();
            ROS_ERROR("[ Data Cut ] Throw one image frame!");
            return false;
        }

        dropStaleAuxiliaryMeasurements(meas.last_lio_update_time);
        if (is_first_frame)
        {
            const double next_aux_time = std::min(nextWheelTime(), nextGNSSTime());
            if (next_aux_time + 1e-6 < img_capture_time)
            {
                if (nextWheelTime() <= nextGNSSTime())
                    return prepareWheelEvent(meas, next_aux_time);
                return prepareGNSSEvent(meas, next_aux_time);
            }
        }

        if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
            return false;

        MeasureGroup m;
        m.event_time = img_capture_time;
        m.lio_time = img_capture_time;
        m.imu.clear();
        mtx_buffer.lock();
        while (!imu_buffer.empty())
        {
            if (imu_buffer.front()->header.stamp.toSec() > m.lio_time)
                break;
            if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time)
                m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        mtx_buffer.unlock();
        sig_buffer.notify_all();

        *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
        PointCloudXYZI().swap(*meas.pcl_proc_next);

        int lid_frame_num = lid_raw_data_buffer.size();
        int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
        meas.pcl_proc_cur->reserve(max_size);
        meas.pcl_proc_next->reserve(max_size);

        while (!lid_raw_data_buffer.empty())
        {
            if (lid_header_time_buffer.front() > img_capture_time)
                break;
            auto pcl(lid_raw_data_buffer.front()->points);
            double frame_header_time(lid_header_time_buffer.front());
            float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

            for (int i = 0; i < pcl.size(); i++)
            {
                auto pt = pcl[i];
                if (pcl[i].curvature < max_offs_time_ms)
                {
                    pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
                    meas.pcl_proc_cur->points.push_back(pt);
                }
                else
                {
                    pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
                    meas.pcl_proc_next->points.push_back(pt);
                }
            }
            lid_raw_data_buffer.pop_front();
            lid_header_time_buffer.pop_front();
        }

        meas.measures.clear();
        meas.measures.push_back(m);
        meas.lio_vio_flg = LIO;
        return true;
    }

    case ONLY_LO:
    {
        if (!lidar_pushed)
        {
            if (lid_raw_data_buffer.empty())
                return false;
            meas.lidar = lid_raw_data_buffer.front();
            meas.lidar_frame_beg_time = lid_header_time_buffer.front();
            meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_pushed = true;
        }
        MeasureGroup m;
        m.event_time = meas.lidar_frame_end_time;
        m.lio_time = meas.lidar_frame_end_time;
        mtx_buffer.lock();
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        lidar_pushed = false;
        meas.lio_vio_flg = LO;
        meas.measures.clear();
        meas.measures.push_back(m);
        return true;
    }

    default:
        printf("!! WRONG SLAM TYPE !!");
        return false;
    }
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
    cv::Mat img_rgb = vio_manager->img_cp;
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    // out_msg.header.frame_id = "camera_init";
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = img_rgb;
    pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
    if (pcl_w_wait_pub->empty())
        return;
    PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
    static int pub_num = 1;
    pub_num++;

    if (LidarMeasures.lio_vio_flg == VIO)
    {
        *pcl_wait_pub += *pcl_w_wait_pub;
        if (pub_num >= pub_scan_num)
        {
            pub_num = 1;
            size_t size = pcl_wait_pub->points.size();
            laserCloudWorldRGB->reserve(size);
            // double inv_expo = _state.inv_expo_time;
            cv::Mat img_rgb = vio_manager->img_rgb;
            for (size_t i = 0; i < size; i++)
            {
                PointTypeRGB pointRGB;
                pointRGB.x = pcl_wait_pub->points[i].x;
                pointRGB.y = pcl_wait_pub->points[i].y;
                pointRGB.z = pcl_wait_pub->points[i].z;

                V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
                V3D pf(vio_manager->new_frame_->w2f(p_w));
                if (pf[2] < 0)
                    continue;
                V2D pc(vio_manager->new_frame_->w2c(p_w));

                if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
                {
                    V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
                    pointRGB.r = pixel[2];
                    pointRGB.g = pixel[1];
                    pointRGB.b = pixel[0];
                    // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
                    // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
                    // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
                    // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
                    if (pf.norm() > blind_rgb_points)
                        laserCloudWorldRGB->push_back(pointRGB);
                }
            }
        }
    }

    /*** Publish Frame ***/
    sensor_msgs::PointCloud2 laserCloudmsg;
    if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
    {
        pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
    }
    if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
    {
        pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
    }
    laserCloudmsg.header.stamp = ros::Time().fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    double update_time = 0.0;
    if (LidarMeasures.lio_vio_flg == VIO)
    {
        update_time = LidarMeasures.measures.back().vio_time;
    }
    else
    { // LIO / LO
        update_time = LidarMeasures.measures.back().lio_time;
    }
    std::stringstream ss_time;
    ss_time << std::fixed << std::setprecision(6) << update_time;

    if (pcd_save_en)
    {
        static int scan_wait_num = 0;

        switch (pcd_save_type)
        {
        case 0: /** world frame **/
            if (slam_mode_ == LIVO)
            {
                *pcl_wait_save += *laserCloudWorldRGB;
            }
            else
            {
                *pcl_wait_save_intensity += *pcl_w_wait_pub;
            }
            if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
                scan_wait_num++;
            break;

        case 1: /** body frame **/
            if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
            {
                int size = feats_undistort->points.size();
                PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
                for (int i = 0; i < size; i++)
                {
                    RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
                }
                *pcl_wait_save_intensity += *laserCloudBody;
                scan_wait_num++;
                cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
            }
            pcd_save_interval = 1;

            break;

        default:
            pcd_save_interval = 1;
            scan_wait_num++;
            break;
        }
        if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            string all_points_dir(run_output_dir_ + "/pcd/" + ss_time.str() + string(".pcd"));

            pcl::PCDWriter pcd_writer;

            cout << "current scan saved to " << all_points_dir << endl;
            if (pcl_wait_save->points.size() > 0)
            {
                pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
                PointCloudXYZRGB().swap(*pcl_wait_save);
            }
            if (pcl_wait_save_intensity->points.size() > 0)
            {
                pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
                PointCloudXYZI().swap(*pcl_wait_save_intensity);
            }
            scan_wait_num = 0;
        }

        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
            Eigen::Quaterniond q(_state.rot_end);
            fout_lidar_pos << std::fixed << std::setprecision(6);
            fout_lidar_pos << LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
                           << " " << q.w() << " " << endl;
        }
    }
    if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
    {
        static int img_wait_num = 0;
        img_wait_num++;

        if (img_save_interval > 0 && img_wait_num >= img_save_interval)
        {
            imwrite(run_output_dir_ + "/image/" + ss_time.str() + string(".png"), vio_manager->img_rgb);

            Eigen::Quaterniond q(_state.rot_end);
            fout_visual_pos << std::fixed << std::setprecision(6);
            fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
                            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            img_wait_num = 0;
        }
    }

    if (laserCloudWorldRGB->size() > 0)
        PointCloudXYZI().swap(*pcl_wait_pub);
    if (LidarMeasures.lio_vio_flg == VIO)
        PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
    int size = laserCloudFullRes->points.size();
    if (size == 0)
        return;
    PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
    *sub_pcl_visual_map_pub = *laserCloudFullRes;
    if (1)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time::now();
        laserCloudmsg.header.frame_id = "camera_init";
        pubSubVisualMap.publish(laserCloudmsg);
    }
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
    int effect_feat_num = ptpl_list.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
    for (int i = 0; i < effect_feat_num; i++)
    {
        laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
        laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
        laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time::now();
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T>
void LIVMapper::set_posestamp(T &out)
{
    out.position.x = _state.pos_end(0);
    out.position.y = _state.pos_end(1);
    out.position.z = _state.pos_end(2);
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time().fromSec(last_timestamp_lidar);
    set_posestamp(odomAftMapped.pose.pose);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
    q.setW(geoQuat.w);
    q.setX(geoQuat.x);
    q.setY(geoQuat.y);
    q.setZ(geoQuat.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));
    pubOdomAftMapped.publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
    msg_body_pose.header.stamp = ros::Time().fromSec(last_timestamp_lidar);
    msg_body_pose.header.frame_id = "camera_init";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}
