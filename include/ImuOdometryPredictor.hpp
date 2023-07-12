
#ifndef _IMU_ODOMETRY_PREDICTOR_H_
#define _IMU_ODOMETRY_PREDICTOR_H_

#include "CommonUtility.hpp"


class ImuOdometryPredictor
{
public:
    struct Options {
        float imuRate = 500;
        float imuAccNoise = 3.9939570888238808e-03;
        float imuGyrNoise = 1.5636343949698187e-03;
        float imuAccBiasN = 6.4356659353532566e-05;
        float imuGyrBiasN = 3.5640318696367613e-05;
        float imuGravity = 9.80511;
    };

public:
    ImuOdometryPredictor(const Options& options);

public:
    bool reset(const EntityPose& odomState, bool degenerate);
    bool predict(const EntityPose& imuSample, EntityPose& imuState);

protected:
    gtsam::NavState predictOdometry(const gtsam::NavState& prevState, const gtsam::imuBias::ConstantBias& prevBias, const EntityPose& imuSample);
    bool resetImuIntegrator(const gtsam::imuBias::ConstantBias& prevBias, const std::vector<EntityPose>& imuSamples=std::vector<EntityPose>(), double lastImuTime=-1);
    void resetGraph();
    void resetStatus();
    bool optimizeOdometry(const gtsam::NavState& odomState, const std::vector<EntityPose>& imuSamples, bool degenerate);
    bool startOptimize(const gtsam::NavState& odomStatee, const std::vector<EntityPose>& imuSamples);
    bool restartOptimize();
    bool processOptimize(const gtsam::NavState& odomState, const std::vector<EntityPose>& imuSamples, bool degenerate);
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);

protected:
    Options options_; // 算法参数

    std::deque<EntityPose> imuQueue_;

    bool systemInitialized_; // 标志系统初始化，主要是用来初始化gtsam
    bool doneFirstOpt_; // 第一帧初始化标签
    double lastImuTimeOpt_; // 上一个IMU数据的时间（在雷达里程计的handler中使用）
    double lastImuTimePrd_;
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

    // imuIntegratorOpt_负责预积分两帧激光里程计之间的IMU数据，计算IMU的bias
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorOpt_;
    // imuIntegratorPrd_根据最新的激光里程计，以及后续到到的IMU数据，预测从当前激光里程计往后的位姿（IMU里程计）
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorPrd_;

    // 因子图优化过程中的状态变量
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::NavState odomState_;
    gtsam::imuBias::ConstantBias odomBias_;
};

#endif // _IMU_ODOMETRY_PREDICTOR_H_


