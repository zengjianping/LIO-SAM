#pragma once
#ifndef _COMMON_UTILITY_H_
#define _COMMON_UTILITY_H_
#define PCL_NO_PRECOMPILE 

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 

#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>


struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
    (float, x, x) (float, y, y)
    (float, z, z) (float, intensity, intensity)
    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
    (double, time, time))

typedef pcl::PointXYZI PointType;

typedef PointXYZIRPYT PointTypePose;

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);

inline gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

inline gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

inline Eigen::Affine3f pclPointToAffine3f(const PointTypePose& thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

inline Eigen::Isometry3d pclPointToIsometry3d(const PointTypePose& thisPoint)
{ 
    Eigen::Affine3f affine3f = pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    return Eigen::Isometry3d(affine3f.matrix().cast<double>());
}

inline PointTypePose Isometry3dToPclPoint(const Eigen::Isometry3d& isometery3d)
{
    PointTypePose thisPoint;
    Eigen::Affine3f affine3f(isometery3d.matrix().cast<float>());
    pcl::getTranslationAndEulerAngles(affine3f, thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    return thisPoint;
}

inline void affine3fToTrans(const Eigen::Affine3f& affinePose, float transformIn[])
{
    pcl::getTranslationAndEulerAngles(affinePose, transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

inline Eigen::Affine3f transToAffine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

inline Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

inline gtsam::Pose3 poseEigen2Gtsam(const Eigen::Isometry3d& eigenPose)
{
    return gtsam::Pose3(eigenPose.matrix());
}

PointTypePose trans2PointTypePose(float transformIn[]);

inline float pointDistance(const PointTypePose& p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline float pointDistance(const PointTypePose& p1, const PointTypePose& p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

inline float pointDistance(const PointType& p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline float pointDistance(const PointType& p1, const PointType& p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

inline PointType pose3DFromPose6D(const PointTypePose& pose6D)
{
    PointType pose3D;
    pose3D.x = pose6D.x;
    pose3D.y = pose6D.y;
    pose3D.z = pose6D.z;
    pose3D.intensity = pose6D.intensity;
}


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


class ImuSample
{
public:
    ImuSample();

public:
    double timestamp_;
    Eigen::Vector3d linearAcceleration_;
    Eigen::Vector3d angularVelocity_;
    Eigen::Vector3d angularRPY_;
};

class PoseSample
{
public:
    PoseSample();

public:
    double timestamp_;
    Eigen::Vector3d position_;
    Eigen::Vector3d angularRPY_;
    boost::array<double, 36> covariance_;
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
    PointType pose3D; // 三维位姿，x,y,z
    PointTypePose pose6D; // 六维位姿，x,y,z,roll,pitch,yaw
    pcl::PointCloud<PointType>::Ptr extractedCloud; // 
    pcl::PointCloud<PointType>::Ptr cornerCloud; // 角点点云
    pcl::PointCloud<PointType>::Ptr surfCloud; // 平面点点云
};
typedef std::vector<MapPoseFrame> MapPoseFrameVec;
typedef boost::shared_ptr<MapPoseFrameVec> MapPoseFrameVecPtr;

#endif // _COMMON_UTILITY_H_


