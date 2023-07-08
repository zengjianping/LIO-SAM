
#ifndef __LASER_LOOP_DETECTOR_H__
#define __LASER_LOOP_DETECTOR_H__

#include "CommonUtility.hpp"
#include "Scancontext.h"


class LaserLoopDetector
{
public:
    struct Options {
        float mappingIcpLeafSize;
        float historyKeyframeSearchRadius;
        float historyKeyframeSearchTimeDiff;
        int historyKeyframeSearchNum;
        float historyKeyframeFitnessScore;
        bool enableScanContextLoopClosure;
    };

public:
    LaserLoopDetector(const Options& options);

public:
    bool process(const pcl::PointCloud<PointType>::Ptr& cloudKeyPoses3D, const pcl::PointCloud<PointTypePose>::Ptr& cloudKeyPoses6D,
            const vector<pcl::PointCloud<PointType>::Ptr>& cornerCloudKeyFrames, const vector<pcl::PointCloud<PointType>::Ptr>& surfCloudKeyFrames,
            vector<pair<int, int>>& loopIndexQueue, vector<gtsam::Pose3>& loopPoseQueue, vector<gtsam::SharedNoiseModel>& loopNoiseQueue,
            double laserTime, std::pair<double,double>* loopInfo=nullptr);

public:
    SCManager scManager; // scancontext loop closure

protected:
    bool performRSLoopClosure(std::pair<double,double>* loopInfo);
    bool performSCLoopClosure();
    bool detectLoopClosureDistance(int *latestID, int *closestID);
    bool detectLoopClosureExternal(std::pair<double,double>* loopInfo, int *latestID, int *closestID);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int loop_index=-1);

protected:
    Options options_; // 算法参数

    // 当前雷达帧信息
    double timeLaserInfoCur; // 当前雷达帧的时间戳，秒

    // 保存所有关键帧的三维位姿，x,y,z
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    // 保存所有关键帧的六维位姿，x,y,z,roll,pitch,yaw
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    // 全局地图关键帧点云
    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 所有关键帧的角点点云
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // 所有关键帧的平面点点云

    // 点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterICP; // 做回环检测时使用ICP时的点云降采样器

    // 回环检测数据
    std::map<int, int> loopIndexContainer; // 回环的索引字典，从当前帧到回环节点的索引
    std::vector<pair<int, int>> loopIndexQueue; // 所有回环配对关系
    std::vector<gtsam::Pose3> loopPoseQueue; // 所有回环的姿态配对关系
    std::vector<gtsam::SharedNoiseModel> loopNoiseQueue; // 每个回环因子的噪声模型
};

#endif // __LASER_LOOP_DETECTOR_H__


