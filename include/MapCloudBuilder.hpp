
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

        double mappingProcessInterval;
        double mappingIntervalTime = -1.0;

        // Loop closure
        bool enableScanContextLoopClosure = false;
        int historyKeyframeSearchNum;

        // voxel filter paprams
        float mappingCornerLeafSize;
        float mappingSurfLeafSize ;

        // Surrounding map
        float surroundingkeyframeAddingDistThreshold; 
        float surroundingkeyframeAddingAngleThreshold; 
        float surroundingKeyframeDensity;
        float surroundingKeyframeSearchRadius;

        // Save pcd
        bool savePCD;
        string savePCDDirectory;
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

public:
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

    // 全局地图关键帧点云
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 所有关键帧的角点点云
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // 所有关键帧的平面点点云
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer; // 变换到odom坐标系下的关键帧点云字典，为了加速缓存历史关键帧变换后的点云

    /**
     * cloudKeyPoses3D保存所有关键帧的三维位姿，x,y,z
     * cloudKeyPoses6D保存所有关键帧的六维位姿，x,y,z,roll,pitch,yaw
     * 带copy_前缀的两个位姿序列是在回环检测线程中使用的，只是为了不干扰主线程计算，实际内容一样。
    */
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    // 回环检测数据
    vector<pair<int, int>> loopIndexQueue; // 所有回环配对关系
    vector<gtsam::Pose3> loopPoseQueue; // 所有回环的姿态配对关系
    vector<gtsam::SharedNoiseModel> loopNoiseQueue; // 每个回环因子的噪声模型
};

#endif // __MAP_CLOUD_BUILDER_H__


