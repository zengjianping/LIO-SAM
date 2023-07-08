
#ifndef __MAP_CLOUD_BUILDER_H__
#define __MAP_CLOUD_BUILDER_H__

#include "CommonUtility.hpp"
#include "LaserCloudExtractor.hpp"
#include "LaserCloudRegister.hpp"
#include "LaserLoopDetector.hpp"
#include "MapPoseOptimizer.hpp"


class MapCloudBuilder
{
public:
    struct Options {
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
    bool processLaserCloud(const pcl::PointCloud<PointXYZIRT>::Ptr laserCloud, double laserTime, std::vector<PoseSample>& gpsSamples);
    void performLoopClosure(std::pair<double,double>* loopInfo);
    bool saveCloudMap(const string& dataDir, float mapResolution);

protected:
    void ExtractPointCloud();

    void updateInitialGuess();

    void extractSurroundingKeyFrames();
    void extractNearby();
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void extractForLoopClosure();

    void downsampleCurrentScan();

    void scan2MapOptimization();

    void saveKeyFramesAndFactor(std::vector<PoseSample>& gpsSamples);
    bool saveFrame();

protected:
    Options options_; // 算法参数
    std::mutex mtxCloud; // 点云信息回调函数锁

    boost::shared_ptr<LaserCloudExtractor> laserCloudExtractor_;
    boost::shared_ptr<LaserCloudRegister> laserCloudRegister_;
    boost::shared_ptr<LaserLoopDetector> laserLoopDetector_;
    boost::shared_ptr<MapPoseOptimizer> mapPoseOptimizer_;

    // 当前雷达帧信息
    double timeLaserInfoCur; // 当前雷达帧的时间戳，秒
    double timeLastProcessing = -1;
    Eigen::Isometry3d laserPoseGuess;
    Eigen::Isometry3d laserPoseCurr;
    Eigen::Isometry3d laserPoseLast;

    // 当前雷达帧点云
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr extractedCloud; // 当前雷达帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    // 局部地图关键帧点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap; // 局部地图角点点云（odom坐标系）
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS; // 局部地图角点点云降采样
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap; // 局部地图平面点点云（odom坐标系）
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS; // 局部地图平面点点云降采样

    // 点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterCorner; // 角点点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterSurf; // 平面点点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    // 全局地图关键帧数据
    MapPoseFrameVecPtr mapPoseKeyFrames; // 地图关键帧位姿数据
    //std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 所有关键帧的角点点云
    //std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // 所有关键帧的平面点点云
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer; // 变换到odom坐标系下的关键帧点云字典，为了加速缓存历史关键帧变换后的点云
    LoopClosureItemVecPtr loopClosureItems; // 所有回环配对
};

#endif // __MAP_CLOUD_BUILDER_H__


