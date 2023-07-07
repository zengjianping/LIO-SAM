
#ifndef __MAP_BUILDING_H__
#define __MAP_BUILDING_H__

#include "CoreUtility.hpp"
#include "Scancontext.h"

using namespace gtsam;


enum class SCInputType 
{ 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 


class MapPoseOptimizer
{
public:
    struct Options {
        // GPS Settings
        bool useGpsElevation;
        float gpsCovThreshold;
        float poseCovThreshold;
    };

public:
    MapPoseOptimizer(const Options& options);
    ~MapPoseOptimizer();

    bool process(pcl::PointCloud<PointType>::Ptr& cloudKeyPoses3D, pcl::PointCloud<PointTypePose>::Ptr& cloudKeyPoses6D,
        double laserTime, const Eigen::Isometry3d& odomPose, std::vector<PoseSample>& gpsSamples,
        vector<pair<int, int>>& loopIndexQueue, vector<gtsam::Pose3>& loopPoseQueue, vector<gtsam::SharedNoiseModel>& loopNoiseQueue);
    bool poseUpdated() { return aLoopIsClosed; }

protected:
    void addOdomFactor(const Eigen::Isometry3d& odomPose);
    void addGPSFactor(std::vector<PoseSample>& gpsSamples);
    void addLoopFactor(vector<pair<int, int>>& loopIndexQueue, vector<gtsam::Pose3>& loopPoseQueue,
            vector<gtsam::SharedNoiseModel>& loopNoiseQueue);

public:
    Options options_; // 算法参数

    // 因子图优化数据
    ISAM2 *isam = nullptr; // 非线性优化器
    NonlinearFactorGraph gtSAMgraph; // 因子图
    Values initialEstimate; // 因子图变量初始值
    Eigen::MatrixXd poseCovariance; // 当前优化结果的位姿方差，该方差在GPS因子中用到，如果该方差较小，则说明优化结果较好，即使打开GPS开关也不会将GPS因子加入因子图。

    // 回环检测数据
    bool aLoopIsClosed = false;
    PointType lastGPSPoint; // last gps position

    double timeLaserInfoCur; // 当前雷达帧的时间戳，秒
    Eigen::Isometry3d lastPose;

    // cloudKeyPoses3D保存所有关键帧的三维位姿，x,y,z
    // cloudKeyPoses6D保存所有关键帧的六维位姿，x,y,z,roll,pitch,yaw
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
};

#endif // __MAP_BUILDING_H__


