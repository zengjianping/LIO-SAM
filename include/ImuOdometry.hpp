
#ifndef _IMU_ODOMETRY_H_
#define _IMU_ODOMETRY_H_

#include "CoreUtility.hpp"


class ImuOdometryIntegrator
{
public:
    ImuOdometryIntegrator(const SystemParameter& params);

public:
    bool reset(const gtsam::imuBias::ConstantBias& prevBias, const std::vector<ImuSample>& imuSamples=std::vector<ImuSample>(), double lastImuTime=-1);
    gtsam::NavState predict(const gtsam::NavState& prevState, const gtsam::imuBias::ConstantBias& prevBias, const ImuSample& imuSample);

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
    bool odometryInitialized() {return doneFirstOpt_;}
    const gtsam::NavState& getOdomeryState() {return odomState_;}
    const gtsam::imuBias::ConstantBias& getOdometryBias() {return odomBias_;}

protected:
    void resetGraph();
    void resetStatus();
    bool startOptimize(const gtsam::Pose3& lidarPose, const std::vector<ImuSample>& imuSamples);
    bool restartOptimize();
    bool processOptimize(const gtsam::Pose3& lidarPose, const std::vector<ImuSample>& imuSamples, bool degenerate);
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);

protected:
    SystemParameter params_; // 算法参数

    bool systemInitialized_; // 标志系统初始化，主要是用来初始化gtsam
    bool doneFirstOpt_; // 第一帧初始化标签
    double lastImuTime_; // 上一个IMU数据的时间（在雷达里程计的handler中使用）
    int key_; // node key of factor graph

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

    //gtsam::Pose3 imu2Lidar_; // tramsform points from lidar frame to imu frame
    //gtsam::Pose3 lidar2Imu_; // tramsform points from imu frame to lidar frame

    // 因子图优化过程中的状态变量
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::NavState odomState_;
    gtsam::imuBias::ConstantBias odomBias_;
};

#endif // _IMU_ODOMETRY_H_


