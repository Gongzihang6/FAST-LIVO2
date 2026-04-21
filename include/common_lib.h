/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <utils/so3_math.h>
#include <utils/types.h>
#include <utils/color.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <sophus/se3.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace Sophus;

// C++ 调试宏。只要在代码里写一句 print_line;，终端就会打印出当前执行到了哪个文件（__FILE__）的第几行（__LINE__）。
// 用来追踪程序崩溃（段错误）位置或检查代码执行流非常方便。
#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;
#define G_m_s2 (9.81)	 // Gravaty const in GuangDong/China
#define DIM_STATE (19)	 // Dimension of states (Let Dim(SO(3)) = 3) 位置、旋转、速度、陀螺仪零偏、加速度计零偏、重力加速度、多传感器的时间偏差
#define INIT_COV (0.01)	 // 初始状态的信任程度，所有状态量的标准差都设置为0.01
#define SIZE_LARGE (500) // 定义一些数组、缓冲区（Buffer）或特征点提取数量的魔法数字
#define SIZE_SMALL (100)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2] // 语法糖，方便做赋值
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))
// 定义系统保存日志、轨迹、运行时间统计等 Debug 文件的默认保存路径。强制指向当前工程 ROOT_DIR 下的 Log/ 文件夹

/**
 * 定义系统支持的激光雷达硬件类型
 * 不同雷达的数据包格式、时间戳排列（如单点时间戳还是整帧时间戳）、扫描线束提取方式完全不同。
 * 系统前端预处理（Preprocess）代码会根据这个枚举值，调用相应的雷达数据解包和特征提取函数，将它们统一转换成标准 PCL 点云格式
 */
enum LID_TYPE
{
	AVIA = 1,	   // Livox Avia（固态雷达，非重复扫描，FAST-LIO系列常用）
	VELO16 = 2,	   // Velodyne 16线（传统机械旋转雷达）
	OUST64 = 3,	   // Ouster 64线（机械旋转雷达）
	L515 = 4,	   // Intel RealSense L515（固态激光雷达/RGB-D相机）
	XT32 = 5,	   // Hesai XT32（禾赛32线机械雷达）
	PANDAR128 = 6, // Hesai Pandar 128线
	ROBOSENSE = 7  // 速腾聚创雷达
};

// 允许系统在不同传感器配置下灵活运行。如果未订阅到图像或配置关闭，系统可以降级运行
enum SLAM_MODE
{
	ONLY_LO = 0,  // 仅激光里程计 (LiDAR-Only Odometry)，不使用IMU和视觉
	ONLY_LIO = 1, // 激光惯性里程计 (LiDAR-Inertial Odometry)，退化为类似 FAST-LIO2 的模式
	LIVO = 2	  // 激光惯性视觉里程计 (LiDAR-Inertial-Visual Odometry)，系统的全功能模式，融合激光雷达、IMU和视觉数据
};

// 定义 ESIKF（误差状态迭代 卡尔曼滤波）当前处于哪种测量更新状态。
enum EKF_STATE
{
	WAIT = 0, // 等待状态（数据未准备好或初始化中）
	VIO = 1,  // 视觉惯性里程计更新阶段（处理图像残差）
	LIO = 2,  // 激光惯性里程计更新阶段（处理点面匹配残差）
	LO = 3	  // 纯激光更新阶段
};

struct MeasureGroup
{
	double vio_time;					   // 当前视觉帧(Image)对应的时间戳
	double lio_time;					   // （在纯LIO模式下可能用于记录时间，但通常由 LidarMeasureGroup 接管）
	deque<sensor_msgs::Imu::ConstPtr> imu; // 双端队列，存储在此视觉帧时间窗口内接收到的所有 IMU 数据
	cv::Mat img;						   // 当前视觉帧的图像数据（OpenCV 矩阵）
	MeasureGroup()						   // 构造函数，初始化时间戳为0
	{
		vio_time = 0.0;
		lio_time = 0.0;
	};
};

struct LidarMeasureGroup
{
	double lidar_frame_beg_time;		 // 当前这一帧激光雷达扫描的起始时间戳
	double lidar_frame_end_time;		 // 当前这一帧激光雷达扫描的结束时间戳（通常是最后一个点的时间戳）
	double last_lio_update_time;		 // 上一次完成 LIO 状态更新的时间戳
	PointCloudXYZI::Ptr lidar;			 // 存储当前帧原始的激光雷达点云（XYZ + Intensity）
	PointCloudXYZI::Ptr pcl_proc_cur;	 // 预处理后（如去畸变、降采样）准备用于当前优化的点云
	PointCloudXYZI::Ptr pcl_proc_next;	 // （有时用于缓存或跨帧处理）
	deque<struct MeasureGroup> measures; // 双端队列，存储在当前这帧雷达扫描期间，产生的所有【视觉+IMU】数据包
	EKF_STATE lio_vio_flg;				 // 标记这一个数据包送到后端时，应该执行 VIO 还是 LIO 更新
	int lidar_scan_index_now;			 // 记录当前处理到了第几帧雷达数据

	// 构造函数，对各类指针和时间戳进行初始化清零操作，防止野指针和脏数据
	LidarMeasureGroup()
	{
		lidar_frame_beg_time = -0.0;
		lidar_frame_end_time = 0.0;
		last_lio_update_time = -1.0;
		lio_vio_flg = WAIT;
		this->lidar.reset(new PointCloudXYZI());
		this->pcl_proc_cur.reset(new PointCloudXYZI());
		this->pcl_proc_next.reset(new PointCloudXYZI());
		this->measures.clear(); // 清空同步包队列
		lidar_scan_index_now = 0;
		last_lio_update_time = -1.0;
	};
};

typedef struct pointWithVar
{
	Eigen::Vector3d point_b;		// point in the lidar body frame
	Eigen::Vector3d point_i;		// point in the imu body frame
	Eigen::Vector3d point_w;		// point in the world frame
	Eigen::Matrix3d var_nostate;	// the var removed the state covarience 剥离了“由于系统自身位姿估计不准”带来的那部分协方差，仅保留纯物理测量的协方差分量
	Eigen::Matrix3d body_var;		// 激光点在激光雷达局部坐标系下的固有测量协方差（如激光雷达测距误差、扫描线束间距等因素导致的测量不确定性）
	Eigen::Matrix3d var;			// 投影到世界坐标系后的最终协方差，综合了测量不确定性和系统状态估计不准带来的不确定性
	Eigen::Matrix3d point_crossmat; // 点坐标的反对称矩阵
	Eigen::Vector3d normal;			// 该点在地图中匹配到的局部平面的法向量。用于计算点到面距离
	pointWithVar()
	{
		var_nostate = Eigen::Matrix3d::Zero();
		var = Eigen::Matrix3d::Zero();
		body_var = Eigen::Matrix3d::Zero();
		point_crossmat = Eigen::Matrix3d::Zero();
		point_b = Eigen::Vector3d::Zero();
		point_i = Eigen::Vector3d::Zero();
		point_w = Eigen::Vector3d::Zero();
		normal = Eigen::Vector3d::Zero();
	};
} pointWithVar;

// 包含了在每一帧雷达扫描结束时，系统需要估计的所有物理量和状态变量，以及它们的协方差矩阵。这个结构体是 ESIKF 迭代优化的核心数据结构，代表了系统当前对环境和自身状态的估计。
struct StatesGroup
{
	StatesGroup()
	{
		this->rot_end = M3D::Identity();
		this->pos_end = V3D::Zero();
		this->vel_end = V3D::Zero();
		this->bias_g = V3D::Zero();
		this->bias_a = V3D::Zero();
		this->gravity = V3D::Zero();
		this->inv_expo_time = 1.0;									 // inv_expo_time 是光度曝光补偿系数
		this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV; // 初始状态协方差矩阵，假设各状态量之间初始不相关，且每个状态量的标准差都设置为 INIT_COV（0.01）
		this->cov(6, 6) = 0.00001;
		this->cov.block<9, 9>(10, 10) = MD(9, 9)::Identity() * 0.00001; // 硬件的初始零偏通常很小且稳定，所以把这几项的初始方差压低（0.00001）
	};

	StatesGroup(const StatesGroup &b)
	{
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
		this->vel_end = b.vel_end;
		this->bias_g = b.bias_g;
		this->bias_a = b.bias_a;
		this->gravity = b.gravity;
		this->inv_expo_time = b.inv_expo_time;
		this->cov = b.cov;
	};

	StatesGroup &operator=(const StatesGroup &b)
	{
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
		this->vel_end = b.vel_end;
		this->bias_g = b.bias_g;
		this->bias_a = b.bias_a;
		this->gravity = b.gravity;
		this->inv_expo_time = b.inv_expo_time;
		this->cov = b.cov;
		return *this;
	};

	StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
	{
		StatesGroup a;
		a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
		a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
		a.inv_expo_time = this->inv_expo_time + state_add(6, 0);
		a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);
		a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);
		a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);
		a.gravity = this->gravity + state_add.block<3, 1>(16, 0);

		a.cov = this->cov;
		return a;
	};

	StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
	{
		this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
		this->pos_end += state_add.block<3, 1>(3, 0);
		this->inv_expo_time += state_add(6, 0);
		this->vel_end += state_add.block<3, 1>(7, 0);
		this->bias_g += state_add.block<3, 1>(10, 0);
		this->bias_a += state_add.block<3, 1>(13, 0);
		this->gravity += state_add.block<3, 1>(16, 0);
		return *this;
	};

	Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
	{
		Matrix<double, DIM_STATE, 1> a;
		M3D rotd(b.rot_end.transpose() * this->rot_end);
		a.block<3, 1>(0, 0) = Log(rotd);
		a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
		a(6, 0) = this->inv_expo_time - b.inv_expo_time;
		a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
		a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
		a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
		a.block<3, 1>(16, 0) = this->gravity - b.gravity;
		return a;
	};

	void resetpose()
	{
		this->rot_end = M3D::Identity();
		this->pos_end = V3D::Zero();
		this->vel_end = V3D::Zero();
	}

	M3D rot_end;							  // the estimated attitude (rotation matrix) at the end lidar point
	V3D pos_end;							  // the estimated position at the end lidar point (world frame)
	V3D vel_end;							  // the estimated velocity at the end lidar point (world frame)
	double inv_expo_time;					  // the estimated inverse exposure time (no scale)
	V3D bias_g;								  // gyroscope bias
	V3D bias_a;								  // accelerator bias
	V3D gravity;							  // the estimated gravity acceleration
	Matrix<double, DIM_STATE, DIM_STATE> cov; // states covariance
};

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
				const Matrix<T, 3, 3> &R)
{
	Pose6D rot_kp;
	rot_kp.offset_time = t;
	for (int i = 0; i < 3; i++)
	{
		rot_kp.acc[i] = a(i);
		rot_kp.gyr[i] = g(i);
		rot_kp.vel[i] = v(i);
		rot_kp.pos[i] = p(i);
		for (int j = 0; j < 3; j++)
			rot_kp.rot[i * 3 + j] = R(i, j);
	}
	// Map<M3D>(rot_kp.rot, 3,3) = R;
	return move(rot_kp);
	/**
	 * std::move 将 rot_kp 强制转换为右值引用。
	 * 这意味着在函数返回时，编译器不会拷贝这个包含数组的结构体，而是直接把内存所有权“转移”给接收者，
	 * 避免了不必要的内存拷贝开销，对于高频调用的底层 SLAM 函数至关重要
	 */
}

#endif