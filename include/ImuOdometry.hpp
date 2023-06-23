
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

#include "coreUtility.h"


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
};

ImuSample::ImuSample() :
    timestamp_(0), linearAcceleration_(0,0), angularVelocity_(0,0,0)
{
}


class ImuOdometryIntegrator
{
public:
    ImuOdometryIntegrator(const SystemParameter& params);

public:
    bool reset(const gtsam::imuBias::ConstantBias& prevBias, const std::vector<ImuSample>& imuSamples=std::vector<ImuSample>(), double lastImuTime=-1);
    gtsam::NavState process(const gtsam::NavState& prevState, const gtsam::imuBias::ConstantBias& prevBias, const ImuSample& imuSample);

protected:
    SystemParameter params_; // 算法参数

    // imuIntegratorImu_根据最新的激光里程计，以及后续到到的IMU数据，预测从当前激光里程计往后的位姿（IMU里程计）
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegrator_;
    double lastImuTime_;
};


class ImuOdometryOptimizer
{
public:
    ImuOdometryOptimizer(const SystemParameter& params);

public:
    bool process(const gtsam::Pose3& lidarPose, const std::vector<ImuSample>& imuSamples, bool degenerate);

protected:
    void resetGraph();
    void resetStatus();
    bool startOptimize(const gtsam::Pose3& lidarPose);
    bool restartOptimize();
    bool processOptimize(const gtsam::Pose3& lidarPose, const std::vector<ImuSample>& imuSamples, bool degenerate);
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);

protected:
    SystemParameter params_; // 算法参数

    bool systemInitialized_; // 标志系统初始化，主要是用来初始化gtsam
    bool doneFirstOpt_; // 第一帧初始化标签
    double lastImuTime_; // 上一个IMU数据的时间（在雷达里程计的handler中使用）
    int key_; // node key of factor graph
    const double delta_t = 0; // 在做IMU数据和雷达里程计同步过程中的时间间隔

    // 因子图优化
    gtsam::ISAM2 optimizer_; // 因子图优化器
    gtsam::NonlinearFactorGraph graphFactors_; // 非线性因子图
    gtsam::Values graphValues_; // 因子图变量的值
    
    // priorXXXNoise是因子图中添加第一个因子时指定的噪声
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise_;
    // correctionNoise是小噪声，correctionNoise2是大噪声。在雷达里程计方差大的情况下使用大噪声，反之使用小噪声
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2_;
    // IMU的Bias噪声模型
    gtsam::Vector noiseModelBetweenBias_;

    // imuIntegrator_负责预积分两帧激光里程计之间的IMU数据，计算IMU的bias
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegrator_;

    gtsam::Pose3 imu2Lidar_; // tramsform points from lidar frame to imu frame
    gtsam::Pose3 lidar2Imu_; // tramsform points from imu frame to lidar frame

    // 因子图优化过程中的状态变量
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;
};

