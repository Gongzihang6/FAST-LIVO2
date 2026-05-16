/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_POINT_H_
#define LIVO_POINT_H_

#include "common_lib.h"
#include "frame.h"
#include <boost/noncopyable.hpp>

class Feature;

/// A visual map point on the surface of the scene.
// 继承自boost::noncopyable，向编译器声明禁用该类的拷贝构造函数和赋值操作符
class VisualPoint : boost::noncopyable {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector3d pos_;                //该点在全局世界坐标系下的 3D 坐标 (x, y, z)
    Vector3d normal_;             //该点所在局部表面的法向量
    Matrix3d normal_information_; //法向量的信息矩阵（协方差矩阵的逆）
    //表示系统对这个法向量估计值的“确信程度”。信息量越大，说明法向量越准，在后续联合优化时分配的权重就越高。
    Vector3d previous_normal_; //存储上一次激光雷达点面匹配后更新的法向量
    // 在地图维护过程中，会频繁地在中间或两端删除/插入过期的观测，链表能提供O(1) 的删除性能
    list<Feature *> obs_;        //存储了所有曾经“看”到过这个 3D 点的历史图像特征块的指针
    Eigen::Matrix3d covariance_; //该 3D 点位置自身的不确定度（协方差矩阵）
    bool is_converged_;          //收敛标志位。当该点被激光雷达扫描多次，其法向量和位置的更新量微乎其微时，置为true
    bool is_normal_initialized_; //!< True if the normal is initialized.
    // 在 obs_ 列表的众多历史观测中，系统会挑出一个质量最高的观测作为基准模板，即参考图像块（Reference Patch）。
    // 后续所有新帧的光度误差，都是拿新帧的图像块去和这个 ref_patch 算像素差值
    bool has_ref_patch_;         //!< True if the point has a reference patch.
    Feature *ref_patch;          //!< Reference patch of the point.

    VisualPoint(const Vector3d &pos);
    ~VisualPoint();
    /**
     * @brief 所有历史观测 obs_ 中，寻找最适合作为参考模板的 Feature。
     *        评分标准（Score）通常综合了NCC（归一化互相关）光度相似度以及观测视角差。
     *        视角差越接近当前相机，光照条件越相似，得分越优
     */
    void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;
    void deleteNonRefPatchFeatures();
    // 基础的增删观测接口
    void deleteFeatureRef(Feature *ftr);
    void addFrameRef(Feature *ftr);
    bool getCloseViewObs(const Vector3d &pos, Feature *&obs,
                         const Vector2d &cur_px) const;
};

#endif // LIVO_POINT_H_
