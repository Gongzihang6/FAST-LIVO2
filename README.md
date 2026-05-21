# FAST-LIVO2 (Extended tightly-coupled GNSS & Wheel)

> **FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry**  
> 本代码库在原作者 [Chunran Zheng (郑纯然)](https://github.com/xuankuzcr) 的优秀开源工作基础上，深度扩展了 **GNSS 与 轮速计 (Wheel Odometry)** 的 IESKF 紧耦合框架，专为长走廊退化、光照剧变与严重遮挡等极端场景设计。

---

## 1. 核心原理解析：原始 FAST-LIVO2

FAST-LIVO2 的核心是一个基于 **迭代误差状态卡尔曼滤波 (IESKF, Iterated Error-State Kalman Filter)** 的多模态紧耦合框架。它巧妙地避免了特征提取的开销，采用“直接法”进行状态估计。

### 1.1 状态定义与 IMU 传播 (Propagation)
系统以 IMU 为核心驱动力。状态向量包含：全局位姿 (R, p)、速度 (v)、IMU 零偏 (bg, ba) 以及传感器间的外参。
- **正向传播**：在新的一帧 LiDAR 或 Camera 数据到来前，利用高频 IMU 数据（通常 200Hz+）进行四阶龙格-库塔积分，预测当前的先验状态，并传播误差状态协方差矩阵 $P$。
- **反向去畸变**：同时利用 IMU 积分得到的连续位姿，对 LiDAR 扫描周期内产生的运动畸变进行逐点补偿。

### 1.2 VIO：直接法光度误差更新
与传统的基于 ORB/FAST 特征点的 VIO 不同，FAST-LIVO2 采用 **直接法 (Direct Method)**：
1. **Patch 提取**：在图像金字塔上提取具有一定梯度信息的图像块 (Patch)。
2. **光度误差残差**：利用先验位姿和 LiDAR 深度地图，将上一帧的 3D 点投影到当前图像帧。残差定义为同一 3D 点在两帧图像上的像素灰度差。
3. **优势**：极大地节省了特征提取与匹配的算力，即使在纹理较弱（但有微小梯度）的区域也能保持一定的鲁棒性。

### 1.3 LIO：增量式 VoxelMap 点到面更新
FAST-LIVO2 摒弃了 KD-Tree，创新性地使用了增量式体素地图（VoxelMap）：
1. **点到面几何残差**：将当前帧的 LiDAR 点云投影到 World 坐标系，在 VoxelMap 中寻找最近的平面，构建点到面的法向量投影距离作为观测残差。
2. **高效更新**：VoxelMap 允许以 $O(1)$ 的时间复杂度进行平面拟合与增量更新，大幅提升了建图速度。

---

## 2. 紧耦合扩展：GNSS 与 Wheel 的融合机制

在极其恶劣的场景下（如 M3DGR 数据集中的无纹理长走廊、剧烈光照变化），纯 LIO/VIO 极易发生 Z 轴（高程）漂移或沿走廊方向的滑移。为此，我们在 FAST-LIVO2 的 IESKF 框架内引入了 **事件驱动的顺序更新 (Event-driven Sequential Update)**。

### 2.1 顺序测量更新架构 (Sequential Update)
在标准 IESKF 中，多源观测如果同步组合成一个巨大的观测矩阵 $H$，求逆计算量极大且极易因单个传感器异常导致矩阵奇异。
我们采用了**顺序更新**：
- 当时间戳对齐的队列中弹出一个观测事件时（无论它是 Wheel、GNSS 还是 VIO/LIO），立即构建其专属的小型观测矩阵 $H$ 和残差 $r$。
- 计算卡尔曼增益 $K = P H^T (H P H^T + R)^{-1}$，更新误差状态向量 $\delta x$。
- 使用 Joseph 形式更新协方差矩阵 $P = (I - KH)P(I - KH)^T + K R K^T$，以保证数值稳定性。
- 随后，将更新后的状态作为**下一个传感器观测的先验**。

### 2.2 GNSS 融合：防漂移锚点与 Z 轴退化
GNSS 提供了全局绝对位置约束，但其自身在 XY 平面（特别是城市峡谷中）常存在多径效应带来的巨大跳变噪声。
我们在融合中设计了两大核心机制：
1. **锁定式全局对齐 (Alignment Lock)**：
   - 将 GNSS 原始的 WGS84 坐标转换为局部的 ENU 坐标。
   - **痛点**：若频繁重估 ENU 到 LIVO World 的对齐矩阵，会对齐矩阵“迎合”里程计自身的漂移，导致 GNSS 失去锚点作用。
   - **解法**：系统仅在车辆起步（移动 >5m）时，收集几十个样本点进行 SVD 分解，求解初始的 `Yaw` 与 `Translation`。一旦求出，永久锁定 (`realign_interval = -1`)，使 GNSS 成为坚如磐石的绝对纠偏锚点。
2. **自适应降维退化 (z_only fallback)**：
   - 在构建观测残差 $r = p_{gnss} - p_{pred}$ 时，进行马氏距离 (Mahalanobis Distance) 卡方检验。
   - **痛点**：GNSS XY 误差大时，全维度观测会被 Gating 机制无情拒绝（Reject），导致高价值的高程 (Z) 数据也被丢弃，无法压制 LIO 的 Z 轴漂移。
   - **解法**：当全维度更新被拒绝时，代码自动提取 $H$ 矩阵的 Z 轴分量，降维构建 1D 的高度观测残差。这保证了在 XY 信号恶劣时，Z 轴依然能被紧紧钉在真实高度上 (`fallback_z_only: true`)。

### 2.3 Wheel 轮速计融合：非完整约束与相对位姿
在室内长走廊（如 Corridor02 序列）中，轮式里程计能提供极高频且极其稳定的局部约束。
1. **位姿/速度模型双轨制**：
   - 优先提取 `/odom` 的 `pose`，计算与上一有效帧的相对位姿差 $\Delta T$。
   - 若位姿差不可靠，则退化为读取轮速计的前向速度 $v_x$。
2. **杆臂补偿 (Lever Arm Compensation)**：
   - 轮速计坐标系中心通常位于后轴中心，而 IMU 位于车顶或雷达内。
   - 在构建残差时，严格根据配置的外参 ($R_{ext}, T_{ext}$)，利用公式 $v_{imu} = v_{wheel} + \omega \times T_{ext}$ 将轮式约束映射至 IMU 状态中心。
3. **非完整约束 (NHC, Non-holonomic Constraint)**：
   - 引入车辆运动学假设：车辆不会发生侧滑与腾空跳跃，即侧向速度 $v_y \approx 0$，垂向速度 $v_z \approx 0$。
   - 在缺乏结构化点云的走廊内，NHC 约束配合前向速度，能死死锁住横向漂移与航向角发散。

---

## 3. 运行与测试

### 3.1 编译要求
- 依赖项：ROS (18.04/20.04), PCL >= 1.8, Eigen >= 3.3.4, OpenCV >= 4.2
- 必须安装 Livox ROS Driver
```bash
cd ~/slam/FAST-LIVO2_ws
catkin_make --pkg fast_livo -j4
source devel/setup.bash
```

### 3.2 针对 M3DGR 数据集的专用配置启动
我们为不同退化场景提供了即插即用的 Launch 文件：

**1. 剧烈光照场景 (Varying-illu, 开启 GNSS + Wheel 紧耦合)**
```bash
roslaunch fast_livo mapping_m3dgr_mid360.launch
rosbag play --clock /path/to/Varying-illu05.bag
```

**2. 长走廊退化场景 (Corridor, 开启 滑动地图)**
```bash
roslaunch fast_livo mapping_m3dgr_corridor.launch
rosbag play --clock /path/to/Corridor02.bag
```

**3. 严重遮挡场景 (Occlusion, 开启 滑动地图)**
```bash
roslaunch fast_livo mapping_m3dgr_occlusion.launch
rosbag play --clock /path/to/Occlusion01.bag
```

*(注：紧耦合特性的开关请在对应的 `config/m3dgr_*.yaml` 中修改 `gnss.enable` 与 `wheel.enable` 字段)*

---

## 4. 相关论文与原作者链接

本代码修改自以下学术成果，请在学术引用中关注原作者：

- [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)  
- [FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)  
- [FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

**Developer**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)  
**Contact**: [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk)  
**License**: [**GPLv2**](http://www.gnu.org/licenses/)