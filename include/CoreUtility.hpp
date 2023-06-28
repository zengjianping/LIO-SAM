#pragma once
#ifndef _CORE_UTILITY_H_
#define _CORE_UTILITY_H_
#define PCL_NO_PRECOMPILE 

#include <Eigen/Core>
#include <Eigen/Dense>

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
using namespace std;

typedef pcl::PointXYZI PointType;

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

inline float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
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


#endif // _CORE_UTILITY_H_


