
#ifndef _COMMON_UTILITY_H_
#define _COMMON_UTILITY_H_

#include "CommonDefine.hpp"


template<typename T>
class Point_
{
public:
    Point_() {
        x_ = T(0);
        y_ = T(0);
        z_ = T(0);
    }

    Point_(T x, T y, T z) {
        x_ = x;
        y_ = y;
        z_ = z;
    }

public:
    T x_, y_, z_;
};

typedef Point_<double> Velocity;
typedef Point_<double> Acceleration;

class EntityPose
{
public:
    EntityPose();
    void init();

    EntityPose(const Eigen::Isometry3d& transform);
    EntityPose(const Eigen::Affine3d& transform);
    EntityPose(const gtsam::Pose3& transform);
    EntityPose(const gtsam::NavState& transform);

    Eigen::Isometry3d toIsometry() const;
    Eigen::Affine3d toAffine() const;
    gtsam::Pose3 toGtsamPose() const;
    gtsam::NavState toGtsamState() const;

    EntityPose inverse() const;
    EntityPose operator *(const EntityPose& other) const;
    EntityPose betweenTo(const EntityPose& other) const;

public:
    double timestamp; // 时间戳
    Eigen::Quaterniond orientation; // 姿态，四元数
    Eigen::Vector3d position; // 位置，x,y,z
    Eigen::Vector3d angular; // 旋转角度，roll, pitch, yaw
    Eigen::Vector3d angularVel; // 旋转角速度
    Eigen::Vector3d linearVel; // 位移速度
    Eigen::Vector3d linearAcc; // 位移加速度
    boost::array<double, 36> covariance; // 位姿协方差
};

// 回环检测配对
class LoopClosureItem
{
public:
    int keyCur=-1, keyPre=-1; // 回环配对关系
    gtsam::Pose3 pose; // 回环的姿态配对
    gtsam::SharedNoiseModel noise; // 回环因子的噪声模型
};
typedef vector<LoopClosureItem> LoopClosureItemVec;
typedef boost::shared_ptr<LoopClosureItemVec> LoopClosureItemVecPtr;

// 地图位姿Frame
class MapPoseFrame
{
public:
    //PointType pose3D; // 三维位姿，x,y,z
    PointTypePose pose6D; // 六维位姿，x,y,z,roll,pitch,yaw
    pcl::PointCloud<PointType>::Ptr extractedCloud; // 过滤的点云
    pcl::PointCloud<PointType>::Ptr cornerCloud; // 角点点云
    pcl::PointCloud<PointType>::Ptr surfCloud; // 平面点点云
};
typedef std::vector<MapPoseFrame> MapPoseFrameVec;
typedef boost::shared_ptr<MapPoseFrameVec> MapPoseFrameVecPtr;

#endif // _COMMON_UTILITY_H_


