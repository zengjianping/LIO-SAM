
#ifndef __LASER_LOOP_DETECTOR_H__
#define __LASER_LOOP_DETECTOR_H__

#include "CommonUtility.hpp"
#include "Scancontext.h"


enum class SCInputType 
{ 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 

class LaserLoopDetector
{
public:
    struct Options {
        float loopClosureICPSurfLeafSize = 0.5;
        float historyKeyframeSearchRadius = 15.0;
        float historyKeyframeSearchTimeDiff = 30.0;
        int historyKeyframeSearchNum = 25;
        float historyKeyframeFitnessScore = 0.3;
        bool enableScanContextLoopClosure = false;
    };

public:
    LaserLoopDetector(const Options& options);

public:
    bool process(double laserCloudTime, MapPoseFrameVecPtr& mapPoseFrames, LoopClosureItemVecPtr& loopClosureItems,
            std::pair<double,double>* loopInfo=nullptr);

protected:
    bool performRSLoopClosure(std::pair<double,double>* loopInfo);
    bool performSCLoopClosure();
    bool detectLoopClosureDistance(int *latestID, int *closestID);
    bool detectLoopClosureExternal(std::pair<double,double>* loopInfo, int *latestID, int *closestID);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int loop_index=-1);

protected:
    Options options_; // 算法参数

    double laserCloudTime_; // 当前雷达帧的时间戳，秒
    MapPoseFrameVecPtr mapPoseFrames_; // 地图位姿数据
    LoopClosureItemVecPtr loopClosureItems_; // 所有回环配对

    std::map<int, int> loopIndexContainer_; // 回环的索引字典，从当前帧到回环节点的索引
    pcl::VoxelGrid<PointType> downSizeFilterICP_; // 做回环检测时使用ICP时的点云降采样器
    SCManager scManager_; // scancontext loop closure
};

#endif // __LASER_LOOP_DETECTOR_H__


