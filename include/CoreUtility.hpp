#ifndef _CORE_UTILITY_H_
#define _CORE_UTILITY_H_

#include "CommonDefine.hpp"

using namespace std;


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

enum class SensorType { VELODYNE, OUSTER, LIVOX, ROBOSENSE, MULRAN };

class SystemParameter
{
public:
    SystemParameter();

public:
    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int point_filter_num;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imuType;
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    float surroundingKeyframeMapLeafSize;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    float loopClosureICPSurfLeafSize;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    bool  enableScanContextLoopClosure;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;
};

class PointCloudInfo
{
public:
    PointCloudInfo();

public:
    double timestamp;

    std::vector<int32_t> startRingIndex;
    std::vector<int32_t> endRingIndex;
    std::vector<int32_t> pointColInd; // point column index in range image
    std::vector<float> pointRange; // point range

    pcl::PointCloud<PointType>::Ptr extractedCloud; // 当前雷达帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr cornerCloud; // 当前雷达帧提取的角点点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud; // 当前雷达帧提取的平面点点云

    bool imuAvailable;
    bool odomAvailable;

    // Attitude for LOAM initialization;
    float imuRollInit;
    float imuPitchInit;
    float imuYawInit;

    // Initial guess from imu pre-integration
    float initialGuessX;
    float initialGuessY;
    float initialGuessZ;
    float initialGuessRoll;
    float initialGuessPitch;
    float initialGuessYaw;
};

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

inline PointTypePose affine3fToPclPoint(const Eigen::Affine3f& affinePose)
{
    PointTypePose thisPoint;
    pcl::getTranslationAndEulerAngles(affinePose, thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    return thisPoint;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn);


#endif // _CORE_UTILITY_H_


