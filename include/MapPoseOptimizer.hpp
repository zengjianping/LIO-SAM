
#ifndef __MAP_POSE_OPTIMIZER_H__
#define __MAP_POSE_OPTIMIZER_H__

#include "CommonUtility.hpp"


class MapPoseOptimizer
{
public:
    struct Options {
        // GPS Settings
        bool useGpsElevation = false;
        float gpsCovThreshold = 2.0;
        float poseCovThreshold = 25.0;
    };

public:
    MapPoseOptimizer(const Options& options);
    ~MapPoseOptimizer();

public:
    bool process(double laserCloudTime, MapPoseFrameVecPtr& mapPoseFrames, LoopClosureItemVecPtr& loopChlosureItems,
            std::vector<EntityPose>& gpsSamples);
    bool poseUpdated() { return aLoopIsClosed_; }

protected:
    void addOdomFactor();
    void addLoopFactor(LoopClosureItemVecPtr& loopClosureItems);
    void addGPSFactor(std::vector<EntityPose>& gpsSamples);

protected:
    Options options_; // 算法参数

    double laserCloudTime_; // 当前雷达帧的时间戳，秒
    MapPoseFrameVecPtr mapPoseFrames_; // 地图位姿数据

    // 因子图优化数据
    gtsam::ISAM2 *isam_ = nullptr; // 非线性优化器
    gtsam::NonlinearFactorGraph gtSAMgraph_; // 因子图
    gtsam::Values initialEstimate_; // 因子图变量初始值
    Eigen::MatrixXd poseCovariance_; // 当前优化结果的位姿方差，该方差在GPS因子中用到，如果该方差较小，则说明优化结果较好，即使打开GPS开关也不会将GPS因子加入因子图。

    bool aLoopIsClosed_ = false; // 回环检测数据
    PointType lastGPSPoint_; // last gps position
};

#endif // __MAP_POSE_OPTIMIZER_H__


