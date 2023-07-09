
#ifndef __MAP_CLOUD_BUILDER_H__
#define __MAP_CLOUD_BUILDER_H__

#include "CommonUtility.hpp"
#include "ImuOdometryPredictor.hpp"
#include "LaserCloudExtractor.hpp"
#include "LaserCloudRegister.hpp"
#include "LaserLoopDetector.hpp"
#include "MapPoseOptimizer.hpp"


class MapCloudBuilder
{
public:
    struct Options {
        ImuOdometryPredictor::Options optionImuOdomPredictor;
        LaserCloudExtractor::Options optionCloudExtractor;
        LaserCloudRegister::Options optionCloudRegister;
        LaserLoopDetector::Options optionLoopDetector;
        MapPoseOptimizer::Options optionPoseOptimizer;

        double mappingProcessInterval = 0.15;
        double mappingIntervalTime = -1.0;

        // voxel filter paprams
        float mappingCornerLeafSize = 0.2;
        float mappingSurfLeafSize = 0.4;

        // Surrounding map
        float surroundingkeyframeAddingDistThreshold = 1.0; 
        float surroundingkeyframeAddingAngleThreshold = 0.2; 
        float surroundingKeyframeDensity = 2.0;
        float surroundingKeyframeSearchRadius = 50.0;

        // Save pcd
        bool savePCD = false;
        string savePCDDirectory = "/Downloads/LOAM/";
    };

public:
    MapCloudBuilder(const Options& options);

public:
    void processLoopClosure();
    bool processLaserCloud(const pcl::PointCloud<PointXYZIRT>::Ptr laserCloud, double laserTime);
    void processImuSample(const EntityPose& imuSample);
    void processGpsSample(const EntityPose& gpsSample);
    void processLoopInfo(const std::pair<double,double>& info);
    bool saveCloudMap(const string& dataDir, float mapResolution);

protected:
    void consumeGpsSamples(double laserTime, std::vector<EntityPose>& gpsSamples);
    bool consumeLoopInfo(std::pair<double,double>& info);

protected:
    void ExtractPointCloud();
    void updateInitialGuess();
    void extractSurroundingKeyFrames();
    void _extractNearby();
    void _extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void scan2MapOptimization();
    bool saveFrame();
    void saveKeyFramesAndFactor();

protected:
    Options options_; // 算法参数

    // 线程锁
    std::mutex mtxCloud_; // 点云信息线程锁
    std::mutex mtxImu_; // IMU线程锁
    std::mutex mtxGps_; // GPS线程锁
    std::mutex mtxLoop_; // 回环检测线程锁

    boost::shared_ptr<ImuOdometryPredictor> imuOdometryPredictor_;
    boost::shared_ptr<LaserCloudExtractor> laserCloudExtractor_;
    boost::shared_ptr<LaserCloudRegister> laserCloudRegister_;
    boost::shared_ptr<LaserLoopDetector> laserLoopDetector_;
    boost::shared_ptr<MapPoseOptimizer> mapPoseOptimizer_;

    // 当前雷达帧信息
    double laserTimeCurr_ = -1; // 当前雷达帧时间戳，单位-秒
    double laserTimePrev_ = -1; // 前一帧雷达帧时间戳
    EntityPose laserPoseGuess_; // 预测位姿
    EntityPose laserPoseCurr_; // 当前位姿
    EntityPose laserPosePrev_; // 上一个位姿

    // 当前雷达帧点云
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn_; // 当前雷达帧原始点云
    pcl::PointCloud<PointType>::Ptr extractedCloud_; // 当前雷达帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS_; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS_; // downsampled surf feature set from odoOptimization
    pcl::VoxelGrid<PointType> downSizeFilterCorner_; // 角点点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterSurf_; // 平面点点云降采样器

    // 局部地图关键帧点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_; // 局部地图角点点云（odom坐标系）
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS_; // 局部地图角点点云降采样
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap_; // 局部地图平面点点云（odom坐标系）
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS_; // 局部地图平面点点云降采样
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses_; // for surrounding key poses of scan-to-map optimization

    // 全局地图关键帧位姿和点云
    MapPoseFrameVecPtr mapPoseKeyFrames_; // 地图关键帧位姿数据
    //std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 所有关键帧的角点点云
    //std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // 所有关键帧的平面点点云
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer_; // 变换到odom坐标系下的关键帧点云字典，为了加速缓存历史关键帧变换后的点云

    // 回环检测数据
    LoopClosureItemVecPtr loopClosureItems_; // 所有回环配对
    std::deque<std::pair<double,double> > loopInfoQueue_; // loop信息队列

    // IMU信息
    std::deque<EntityPose> imuOdomQueue_; // IMU信息队列

    // GPS信息
    std::deque<EntityPose> gpsSampleQueue_; // GPS信息队列
};

#endif // __MAP_CLOUD_BUILDER_H__


