/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/**
 * 为了管理无限大的三维世界，系统不会创建一个巨大的 3D 数组（会内存爆炸），而是把根体素的 3D 坐标 (x, y, z) 压缩成一个 1D 的数字（哈希键值）
 * 定义了哈希函数用到的两个大质数。P 用于乘法错位，N 用于取模防溢出，极大降低哈希碰撞概率
 */
#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig
{
    double max_voxel_size_; // 最大体素尺寸（通常0.5m），把世界切成多大的方块
    int max_layer_;         // 八叉树的最大细分层数
    int max_iterations_;    // ESIKF 优化的最大迭代次数
    std::vector<int> layer_init_num_;   // 每一层触发平面拟合所需的最少点数
    int max_points_num_;    // 一个体素内最多保留多少个点（控制内存）
    double planner_threshold_;  // 判定点云是否构成“平面”的阈值
    double beam_err_;       // 激光雷达的光束测距误差（用于协方差推导）
    double dept_err_;       // 激光雷达的深度误差
    double sigma_num_;      // 马氏距离阈值乘数，用于剔除明显的离群点
    bool is_pub_plane_map_; // 是否在 Rviz 中发布面片地图

    // config of local map sliding
    double sliding_thresh;  // 触发地图滑动的距离阈值
    bool map_sliding_en;    // 是否开启局部地图滑动（边走边删远处的地图）
    int half_map_size;      // 内存中保留的局部地图的一半大小（如100米）
} VoxelMapConfig;

typedef struct PointToPlane
{
    Eigen::Vector3d point_b_;   // 激光点在雷达自身坐标系 (Body) 下的坐标
    Eigen::Vector3d point_w_;   // 激光点在世界坐标系 (World) 下的坐标
    Eigen::Vector3d normal_;    // 匹配到的地图平面的法向量
    Eigen::Vector3d center_;    // 匹配到的地图平面的中心点
    Eigen::Matrix<double, 6, 6> plane_var_; // 该拟合平面的不确定度（协方差）包括平面中心点坐标和法向量的协方差
    M3D body_cov_;  // 激光点自身的物理测量协方差
    int layer_;     // 匹配发生在八叉树的哪一层
    double d_;      // 平面方程 ax+by+cz+d=0 中的 d 值
    double eigen_value_;    // 最小特征值，反映平面的平整度
    bool is_valid_; // 这是一个有效约束吗？（如果太远或太倾斜则置 false）
    float dis_to_plane_;    // 激光点到平面的垂直距离（即我们需要最小化的 残差 Residual）
} PointToPlane;

typedef struct VoxelPlane
{
    Eigen::Vector3d center_;    // 平面中心（点云质心）
    Eigen::Vector3d normal_;    // 平面法向量（PCA求出的最小特征向量）
    Eigen::Vector3d y_normal_;  // 另外两个特征向量，构成平面内的局部坐标系
    Eigen::Vector3d x_normal_;
    Eigen::Matrix3d covariance_;    // 体素内所有点分布的协方差矩阵
    Eigen::Matrix<double, 6, 6> plane_var_; // 平面参数(法矢和距离)的协方差（FAST-LIVO2精细建模）
    float radius_ = 0;  // 平面覆盖的有效半径
    float min_eigen_value_ = 1; // 最小特征值（越小越平整）
    float mid_eigen_value_ = 1;
    float max_eigen_value_ = 1;
    float d_ = 0;   // 平面截距
    int points_size_ = 0;   // 参与拟合这个面的点数
    bool is_plane_ = false; // 这些点真的构成一个面吗？（通过判断最小特征值是否远小于其他两个）
    bool is_init_ = false;  // 是否已经初始化过拟合
    int id_ = 0;    // 平面ID，可能用于关联
    bool is_update_ = false;    // 本帧是否有新点加入导致该平面被更新
    VoxelPlane()
    {
        plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
        covariance_ = Eigen::Matrix3d::Zero();
        center_ = Eigen::Vector3d::Zero();
        normal_ = Eigen::Vector3d::Zero();
    }
} VoxelPlane;

// 三维体素索引与哈希函数
class VOXEL_LOCATION
{
public:
    int64_t x, y, z;

    VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
    template <>
    struct hash<VOXEL_LOCATION>
    {
        int64_t operator()(const VOXEL_LOCATION &s) const
        {
            using std::hash;
            using std::size_t;
            return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
        }
    };
} // namespace std

struct DS_POINT
{
    float xyz[3];
    float intensity;
    int count = 0;
};

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
    VoxelOctoTree() = default;
    std::vector<pointWithVar> temp_points_; // 暂存刚落入此节点的点
    VoxelPlane *plane_ptr_; // 指向提取出的平面参数的指针
    int layer_; // 当前所处的树深度层级
    int octo_state_; // 0 is end of tree, 1 is not
    VoxelOctoTree *leaves_[8];  // 如果分裂，指向 8 个更小的子体素的指针
    double voxel_center_[3]; // x, y, z 该体素的几何中心坐标
    std::vector<int> layer_init_num_;
    float quater_length_;
    float planer_threshold_;
    int points_size_threshold_;
    int update_size_threshold_;
    int max_points_num_;
    int max_layer_;
    int new_points_;
    bool init_octo_;
    bool update_enable_;

    VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
        : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
          planer_threshold_(planer_threshold)
    {
        temp_points_.clear();
        octo_state_ = 0;
        new_points_ = 0;
        update_size_threshold_ = 5;
        init_octo_ = false;
        update_enable_ = true;
        for (int i = 0; i < 8; i++)
        {
            leaves_[i] = nullptr;
        }
        plane_ptr_ = new VoxelPlane;
    }

    ~VoxelOctoTree()
    {
        for (int i = 0; i < 8; i++)
        {
            delete leaves_[i];
        }
        delete plane_ptr_;
    }
    void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);    // 用 PCA 拟合平面
    void init_octo_tree();  // 初始化树
    void cut_octo_tree();   // 核心：分裂（切分）树！
    void UpdateOctoTree(const pointWithVar &pv);

    VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
    VoxelOctoTree *Insert(const pointWithVar &pv);
};

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

class VoxelMapManager
{
public:
    VoxelMapManager() = default;    // 显式声明使用编译器默认生成的无参构造函数
    VoxelMapConfig config_setting_; // 存储体素地图的配置参数（如体素大小、最大层数等）
    int current_frame_id_ = 0;      // 记录当前处理到第几帧数据
    ros::Publisher voxel_map_pub_;  // ROS 的发布者，用于把体素地图发给 Rviz 显示
    // 只在有雷达点落入的物理空间才在内存中开辟树节点，实现了无限扩张的地图与极低内存占用的完美平衡。
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_; // 使用哈希表存储体素地图，每个根体素位置对应一个八叉树节点

    // 一帧点云进来，先去畸变，再用体素滤波降采样（feats_down_body_）。匹配时，需要用系统当前预测的位姿将其投影到全局地图中（feats_down_world_），才能与历史地图寻找重合面
    PointCloudXYZI::Ptr feats_undistort_;   // 去除运动畸变后的当前帧原始特征点云
    PointCloudXYZI::Ptr feats_down_body_;   // 在雷达坐标系下（Body Frame）降采样后的点云
    PointCloudXYZI::Ptr feats_down_world_;  // 转换到世界坐标系下（World Frame）的点云

    M3D extR_;  // 雷达与 IMU 之间的旋转外参
    V3D extT_;  // 雷达与 IMU 之间的平移外参

    // 记录构建残差（搜近邻、算距离）和 ESIKF 滤波器更新所花费的时间，并在终端输出，用于衡量系统是否满足 10Hz 的实时性要求。
    float build_residual_time, ekf_time;
    float ave_build_residual_time = 0.0;
    float ave_ekf_time = 0.0;
    int scan_count = 0;
    StatesGroup state_;     // 保存了当前的系统最优位姿和状态

    /**
     * 系统会以 last_slide_position 为中心维护一个局部包围盒（比如 100m×100m）的地图数据。
     * 当当前位姿与 last_slide_position 之间的距离超过 sliding_thresh（比如 50m）时，触发地图滑动机制，（存疑）
     * 把 last_slide_position 更新为当前位姿，并丢弃掉距离 last_slide_position 超过 half_map_size（比如 50m）的地图数据。
     * 这样就实现了一个以当前位姿为中心、半径为 half_map_size 的局部地图，既保证了地图的实时性和局部性，又避免了内存占用过大。
     * 这个机制对于长时间运行的系统尤为重要，可以防止地图无限制增长导致的内存溢出，同时保持系统对周围环境的有效感知和定位能力。
     * 需要注意的是，地图滑动机制的设计需要考虑到系统的运动速度和地图更新频率，确保在触发滑动时能够及时更新地图数据，避免出现地图与实际环境不一致的情况。
     */
    V3D position_last_;     // 记录上一次优化后的位姿位置，用于实现地图滑动（Map Sliding）。
                            // 当当前位姿与上一次优化后的位姿之间的距离超过一定阈值时，触发地图滑动机制，更新地图中心位置，丢弃过远的地图数据，保持地图的局部性和实时性。
    V3D last_slide_position = {0, 0, 0};    
    geometry_msgs::Quaternion geoQuat_;

    int feats_down_size_;   // 当前帧降采样后特征点的数量
    int effct_feat_num_;    // 实际产生有效点面约束（匹配成功）的特征点数量
    std::vector<M3D> cross_mat_list_;   // 存储点坐标的反对称矩阵（Skew-symmetric Matrix）
    std::vector<M3D> body_cov_list_;    // 存储每个激光点在雷达坐标系下的投影协方差矩阵
    std::vector<pointWithVar> pv_list_;     // Point with Variance List，存储经过初步筛选、准备进行残差计算的点
    std::vector<PointToPlane> ptpl_list_;   // Point-To-Plane List，存储成功在地图中找到匹配平面（Voxel Map 中的平面）的约束对

    VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
        : config_setting_(config_setting), voxel_map_(voxel_map)
    {
        current_frame_id_ = 0;
        feats_undistort_.reset(new PointCloudXYZI());
        feats_down_body_.reset(new PointCloudXYZI());
        feats_down_world_.reset(new PointCloudXYZI());
    };

    // 触发激光点云的 ESIKF 序列更新，将先验状态 state_propagat 结合地图观测进行修正
    void StateEstimation(StatesGroup &state_propagat);

    // 基础数学运算，把局部点云乘以 R 加 t 投影到全局
    void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

    // 构建体素地图
    void BuildVoxelMap();
    // 连接 LIO 和 VIO 的桥梁。当视觉模块（VIO）进行“按需光线投射”拿到深度后，通过这个函数去体素地图里查询对应 3D 坐标的颜色/灰度值，用于计算视觉光度误差。
    V3F RGBFromVoxel(const V3D &input_point);

    // 一帧数据匹配完成后（位姿确认为最优），调用此函数将当前帧的激光点**插入（Insert）**到哈希表中对应的 VoxelOctoTree 节点里
    void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

    // 利用多线程同时对几千个点执行 build_single_residual
    void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

    // 拿一个当前帧的点，去八叉树里找到离它最近的体素块，提取出那个块拟合的平面，计算该点到这个平面的垂直距离（Residual）
    void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                               PointToPlane &single_ptpl);

    void pubVoxelMap();

    // mapSliding 和 clearMemOutOfMap 配合工作，根据传入的 XYZ 边界，把超出这个物理边界的哈希桶清空（erase），防止内存溢出
    void mapSliding();
    void clearMemOutOfMap(const int &x_max, const int &x_min, const int &y_max, const int &y_min, const int &z_max, const int &z_min);

private:
    // 可视化，提取地图中每一个体素块拟合出的小平面，并通过 ROS MarkerArray 在 Rviz 里画出一个个带颜色的圆盘或方块
    void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

    void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                        const Eigen::Vector3d rgb);
    
    // CalcVectQuation 把法向量构成的坐标系转成四元数（ROS 显示面片需要）
    void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

    // mapJet 是伪彩映射算法，根据平面的高度或反射率给点云上色（红橙黄绿蓝）
    void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_