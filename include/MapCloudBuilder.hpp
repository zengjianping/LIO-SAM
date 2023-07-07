
#ifndef __MAP_CLOUD_BUILDER_H__
#define __MAP_CLOUD_BUILDER_H__

#include "CommonUtility.hpp"
#include "LaserCloudExtractor.hpp"
#include "LaserCloudRegister.hpp"
#include "LaserLoopDetector.hpp"
#include "MapPoseOptimizer.hpp"

#include "CoreUtility.hpp"


enum class SCInputType 
{ 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 


class MapCloudBuilder
{
public:
    struct Options {
        LaserCloudRegister::Options optionCloudRegister;
    };

public:
    MapCloudBuilder(const Options& options);

public:
    bool saveCloudMap(const string& dataDir, float mapResolution);
    void performLoopClosure(std::pair<double,double>* loopInfo);
    bool processLaserCloud(const PointCloudInfo& _cloudInfo, const std::vector<PoseSample>& gpsSamples);

protected:
    void allocateMemory();

    void performRSLoopClosure(std::pair<double,double>* loopInfo);
    void performSCLoopClosure();
    bool detectLoopClosureDistance(int *latestID, int *closestID);
    bool detectLoopClosureExternal(std::pair<double,double>* loopInfo, int *latestID, int *closestID);
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int loop_index=-1);

    void updateInitialGuess();

    void extractSurroundingKeyFrames();
    void extractNearby();
    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
    void extractForLoopClosure();

    void downsampleCurrentScan();

    void scan2MapOptimization();
    void transformUpdate();
    float constraintTransformation(float value, float limit);

    void saveKeyFramesAndFactor(const std::vector<PoseSample>& gpsSamples);
    bool saveFrame();
    void addOdomFactor();
    void addGPSFactor(const std::vector<PoseSample>& gpsSamples);
    void addLoopFactor();
    void correctPoses();

public:
    Options options_; // 算法参数

    boost::shared_ptr<LaserCloudRegister> laserCloudRegister_;
    Eigen::Isometry3d currInitPose;
    Eigen::Isometry3d currFinalPose;

    SystemParameter params_;
    PointCloudInfo cloudInfo; // 点云信息
    std::mutex mtxCloud; // 点云信息回调函数锁

    // GPS信息
    PointType lastGPSPoint; // last gps position

    // 当前雷达帧信息
    double timeLaserInfoCur; // 当前雷达帧的时间戳，秒
    double timeLastProcessing = -1;

    // 当前雷达帧点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization
    int laserCloudCornerLastDSNum = 0; // 降采样后当前帧角点点云数量
    int laserCloudSurfLastDSNum = 0; // 降采样后当前帧平面点云数量

    /**
     * cloudKeyPoses3D保存所有关键帧的三维位姿，x,y,z
     * cloudKeyPoses6D保存所有关键帧的六维位姿，x,y,z,roll,pitch,yaw
     * 带copy_前缀的两个位姿序列是在回环检测线程中使用的，只是为了不干扰主线程计算，实际内容一样。
    */
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    // 全局地图关键帧点云
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // 所有关键帧的角点点云
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; // 所有关键帧的平面点点云
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer; // 变换到odom坐标系下的关键帧点云字典，为了加速缓存历史关键帧变换后的点云

    // 局部地图关键帧点云
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap; // 局部地图角点点云（odom坐标系）
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS; // 局部地图角点点云降采样
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap; // 在做点云匹配时构建的角点kdtree
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap; // 局部地图平面点点云（odom坐标系）
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS; // 局部地图平面点点云降采样
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap; // 在做点云匹配时构建的平面点kdtree
    int laserCloudCornerFromMapDSNum = 0; // 降采样后局部地图角点点云数量
    int laserCloudSurfFromMapDSNum = 0; // 降采样后局部地图平面点云数量

    // 点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterCorner; // 角点点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterSurf; // 平面点点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterICP; // 做回环检测时使用ICP时的点云降采样器
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;

    // 在做点云匹配的过程中使用的中间变量
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;
    // 标识点云匹配的结果是否较差，当isDegenerate为true的时候，标识本次的点云匹配结果较差，
    // 会在雷达里程计的协方差中置位，在imuPreintegration中会根据这个标志位选择因子图的噪声模型
    bool isDegenerate = false;

    // 雷达帧位姿信息
    /**
     * 注意注意注意！！这是一个非常重要的变量，transformTobeMapped[6]缓存的是当前帧
     * 的`最新`位姿x,y,z,roll,pitch,yaw。无论在哪个环节，对位姿的更新都会被缓存到这个
     * 变量供给下一个环节使用！！
    */
    float transformTobeMapped[6];
    Eigen::Affine3f transPointAssociateToMap; // 缓存雷达帧位姿用来做点云变换
    Eigen::Affine3f incrementalOdometryAffineFront; // 在每次点云进来时缓存上一次的位姿
    // 当前帧点云优化后的最终位姿，与Front可以算出一个增量，应用到上一次的雷达里程计，计算出当前的雷达里程计，这一步似乎有点多余
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Affine3f incrementalOdometryAffineIncr;

    // 因子图优化数据
    NonlinearFactorGraph gtSAMgraph; // 因子图
    Values initialEstimate; // 因子图变量初始值
    ISAM2 *isam; // 非线性优化器
    Values isamCurrentEstimate; // 优化器当前优化结果
    Eigen::MatrixXd poseCovariance; // 当前优化结果的位姿方差，该方差在GPS因子中用到，如果该方差较小，则说明优化结果较好，即使打开GPS开关也不会将GPS因子加入因子图。

    // 回环检测数据
    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // 回环的索引字典，从当前帧到回环节点的索引
    vector<pair<int, int>> loopIndexQueue; // 所有回环配对关系
    vector<gtsam::Pose3> loopPoseQueue; // 所有回环的姿态配对关系
    //vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue; // 每个回环因子的噪声模型
    vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    SCManager scManager; // scancontext loop closure

    // 记录前一个雷达帧的imu姿态
    Eigen::Affine3f lastImuTransformation;
    bool lastImuPreTransAvailable = false;
    Eigen::Affine3f lastImuPreTransformation;
};

#endif // __MAP_CLOUD_BUILDER_H__

