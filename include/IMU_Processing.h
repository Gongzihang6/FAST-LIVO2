/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include <condition_variable>   // condition_variable 通常用于多线程同步
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>

/**
 * 在 PCL 的标准点云格式（如 PointXYZINormal）中，并没有自带保存“该点时间戳”的字段。
 * FAST-LIO 系列非常聪明地借用（Hack）了 curvature（曲率）这个用不到的 float 字段，来存储每个激光点相对于该帧起始时刻的时间偏移量（offset time）。
 * 这个函数的作用就是按照点云被扫描到的时间先后顺序，对一帧内的所有点进行排序，这是后续进行运动畸变补偿的先决条件
 */
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU Process and undistortion
class ImuProcess
{
public:
    // 为了让系统在使用 new ImuProcess() 在堆上动态创建对象时，依然能保证 16/32 字节的严格内存对齐，避免 SIMD 指令导致的段错误，必须在类的 public 区域加上这个 Eigen 提供的宏。
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    // 当 SLAM 跑飞了或者检测到时间戳跳变时，调用 Reset 清空历史积分状态
    void Reset();
    void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

    // 设置 LiDAR 到 IMU 的外参（旋转和平移），可以通过 ROS 参数服务器加载，也可以通过标定工具（如 Kalibr）标定后手动输入
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4, 4) & T);

    // 设置卡尔曼滤波中**过程噪声协方差矩阵（Q矩阵）**的参数。
    // 也就是告诉滤波器：你的陀螺仪和加速度计有多“不准”，以及它们的零偏漂移（随机游走）速度有多快。这直接影响滤波器对 IMU 积分值的信任度。
    void set_gyr_cov_scale(const V3D &scaler);
    void set_acc_cov_scale(const V3D &scaler);
    void set_gyr_bias_cov(const V3D &b_g);
    void set_acc_bias_cov(const V3D &b_a);
    void set_inv_expo_cov(const double &inv_expo);  // 曝光时间的不确定性

    void set_imu_init_frame_num(const int &num);

    // 一系列功能开关。比如在某些只有雷达的数据集里可以 disable_imu() 退化为纯 LO 模式；或者关闭重力估计、零偏估计
    void disable_imu();
    void disable_gravity_est();
    void disable_bias_est();
    void disable_exposure_est();

    void setOutputDir(const std::string &dir);

    /**
     * 这是整个类的主干函数。每次拿到一个同步好的 LidarMeasureGroup（包含一帧雷达和一串 IMU），就会丢进这个函数。
     * 它内部会先检查 IMU 是否初始化，然后调用积分和去畸变逻辑。stat 是当前系统状态，cur_pcl_un_ 是用来存放处理完毕（Un-distorted）后干净点云的指针
     */
    void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);

    /**
     * 点云去畸变的具体实现。它会利用这 100ms 内的 IMU 数据进行高频的前向积分，推算出在这 100ms 内机器人的连续运动轨迹。然后，遍历雷达帧里的每一个点，
     * 根据点自带的时间戳（前面提到的 curvature），从轨迹中插值出该点对应的瞬间位姿，并反向补偿，把所有点强行统一到该帧起始时刻的静态坐标系下。
     */
    void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

    ofstream fout_imu;
    string output_dir_;
    double IMU_mean_acc_norm;   // 记录一段时间内加速度的平均模长，用于判断是否处于静止状态
    V3D unbiased_gyr;   // 扣除零偏后的纯净角速度

    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double cov_inv_expo;
    double first_lidar_time;    // 记录第一帧雷达的时间戳，用于后续计算 IMU 数据相对于雷达帧的时间偏移
    bool imu_time_init = false; // 标记是否已经初始化了 IMU 时间偏移的计算
    bool imu_need_init = true;  // 标记是否需要进行 IMU 初始化（通常在系统启动的前几帧雷达数据时为 true，完成初始状态估计后置为 false）
    M3D Eye3d;  // 3x3 单位矩阵，常用于初始化旋转矩阵
    V3D Zero3d; // 3x1 零向量，常用于初始化位置、速度、偏置等状态变量
    int lidar_type; // 记录当前使用的激光雷达类型（如 Velodyne HDL-64E、Ouster OS1-128 等），可能会影响去畸变的具体参数设置和处理逻辑

private:
    /**
     * 当系统刚启动时调用。它会收集前 N 帧（比如 20 帧）处于静止状态的 IMU 数据。由于静止，加速度计测到的唯一加速度就是反向的重力。
     * 系统求平均后，算出重力向量的方向，从而确定机器人的初始姿态（Roll 和 Pitch），同时计算出初始的陀螺仪静态零偏。
     */
    void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

    // 如果在没有 IMU 的纯雷达模式下，就用匀速运动模型（Constant Velocity Model）来硬猜前向位姿，并进行去畸变
    void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

    PointCloudXYZI pcl_wait_proc;   // 直接实例化的 PCL 点云对象，存储在栈/类实例内存中。用于暂存等待处理的雷达点云。
    sensor_msgs::ImuConstPtr last_imu;  // 做数值积分（推算位姿）时，我们不仅需要当前的加速度，还需要上一时刻的加速度。这个指针就是用来缓存上一个 IMU 数据包的
    PointCloudXYZI::Ptr cur_pcl_un_;    // 存放经过运动补偿后、所有点都被拉回同一时间基准（雷达帧尾）的纯净点云
    vector<Pose6D> IMUpose; // 雷达扫一圈要 100ms，期间 IMU 可能生成了 50 个位姿。这个数组把这 50 个瞬间的 6-DOF（六自由度）位姿全存下来。
                            // 当要对雷达的第k个点去畸变时，系统就会在这个数组里找离它时间最近的两个位姿进行线性插值，精确计算出该点的激光发射中心在哪

    // 激光雷达和 IMU 之间的外参（旋转和平移），用于把 IMU 测量的加速度和角速度从 IMU 坐标系转换到 LiDAR 坐标系，或者反过来。这是进行 IMU 前向传播和雷达时间同步的基础
    M3D Lid_rot_to_IMU;
    V3D Lid_offset_to_IMU;

    // （中值积分/欧拉积分）：在对 IMU 进行积分推导位姿时，需要用到上一时刻的加速度/角速度（_last）和当前时刻的值来求平均（mean_），以提高数值积分的精度
    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;


    double last_prop_end_time;
    double time_last_scan;
    int init_iter_num = 1, MAX_INI_COUNT = 20;
    bool b_first_frame = true;
    bool imu_en = true;
    bool gravity_est_en = true;
    bool ba_bg_est_en = true;
    bool exposure_estimate_en = true;
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif