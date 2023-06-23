
#include "ImuOdometry.hpp"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)


ImuOdometryIntegrator::ImuOdometryIntegrator(const SystemParameter& params)
{
    params_ = params;

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(params_.imuGravity);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(params_.imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(params_.imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias priorImuBias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
    imuIntegrator_.reset(new gtsam::PreintegratedImuMeasurements(p, priorImuBias)); // setting up the IMU integration for IMU message thread

    lastImuTime_ = -1;
}

bool ImuOdometryIntegrator::reset(const gtsam::imuBias::ConstantBias& prevBias, const std::vector<ImuSample>& imuSamples, double lastImuTime)
{
    // reset bias use the newly optimized bias
    imuIntegrator_->resetIntegrationAndSetBias(prevBias);
    lastImuTime_ = lastImuTime;

    // integrate imu message from the beginning of this optimization
    for (const ImuSample& imuSample : imuSamples)
    {
        // pop and integrate imu data that is between two optimizations
        double dt = (lastImuTime_ < 0) ? (1.0 / params_.imuRate) : (imuSample.timestamp_ - lastImuTime_);
        imuIntegrator_->integrateMeasurement(
            gtsam::Vector3(imuSample.linearAcceleration_.x(), imuSample.linearAcceleration_.y(), imuSample.linearAcceleration_.z()),
            gtsam::Vector3(imuSample.angularVelocity_.x(), imuSample.angularVelocity_.y(), imuSample.angularVelocity_.z()), dt);
        lastImuTime_ = imuSample.timestamp_;
    }

    return true;
}

gtsam::NavState ImuOdometryIntegrator::process(const gtsam::NavState& prevState, const gtsam::imuBias::ConstantBias& prevBias, const ImuSample& imuSample)
{
    double dt = (lastImuTime_ < 0) ? (1.0 / params_.imuRate) : (imuSample.timestamp_ - lastImuTime_);
    lastImuTime_ = imuSample.timestamp_;

    // integrate this single imu message
    imuIntegrator_->integrateMeasurement(
        gtsam::Vector3(imuSample.linearAcceleration_.x(), imuSample.linearAcceleration_.y(), imuSample.linearAcceleration_.z()),
        gtsam::Vector3(imuSample.angularVelocity_.x(), imuSample.angularVelocity_.y(), imuSample.angularVelocity_.z()), dt);

    // predict odometry
    gtsam::NavState currState = imuIntegrator_->predict(prevState, prevBias);
    return currState;
}


ImuOdometryOptimizer::ImuOdometryOptimizer(const SystemParameter& params)
{
    params_ = params;

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(params_.imuGravity);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(params_.imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(params_.imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias priorImuBias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
    imuIntegrator_.reset(new gtsam::PreintegratedImuMeasurements(p, priorImuBias)); // setting up the IMU integration for optimization

    priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise_  = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
    priorBiasNoise_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    correctionNoise2_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m

    float imuAccBiasN = params_.imuAccBiasN, imuGyrBiasN = params_.imuGyrBiasN;
    noiseModelBetweenBias_ = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

    const Eigen::Vector3d& extTrans = params_.extTrans;
    imu2Lidar_ = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    lidar2Imu_ = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    resetStatus();
}

void ImuOdometryOptimizer::resetGraph()
{
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer_ = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors_ = newGraphFactors;

    gtsam::Values NewGraphValues;
    graphValues_ = NewGraphValues;
}

void ImuOdometryOptimizer::resetStatus()
{
    systemInitialized_ = false;
    doneFirstOpt_ = false;
    lastImuTime_ = -1;
    key_ = 1;
}

bool ImuOdometryOptimizer::process(const gtsam::Pose3& lidarPose, const std::vector<ImuSample>& imuSamples, bool degenerate)
{
    if (systemInitialized_ == false) {
        // initialize system
        return startOptimize(lidarPose);
    }
    
    if (key_ == 100) {
        // reset graph for speed
        restartOptimize();
    }

    // integrate imu data and optimize
    return processOptimize(lidarPose, imuSamples, degenerate);
}

bool ImuOdometryOptimizer::startOptimize(const gtsam::Pose3& lidarPose)
{
    // reset graph
    resetGraph();

    // initial pose
    prevPose_ = lidarPose.compose(lidar2Imu_);
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise_);
    graphFactors_.add(priorPose);

    // initial velocity
    prevVel_ = gtsam::Vector3(0, 0, 0);
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise_);
    graphFactors_.add(priorVel);

    // initial bias
    prevBias_ = gtsam::imuBias::ConstantBias();
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise_);
    graphFactors_.add(priorBias);

    // add values
    graphValues_.insert(X(0), prevPose_);
    graphValues_.insert(V(0), prevVel_);
    graphValues_.insert(B(0), prevBias_);

    // optimize once
    optimizer_.update(graphFactors_, graphValues_);
    graphFactors_.resize(0);
    graphValues_.clear();

    imuIntegrator_->resetIntegrationAndSetBias(prevBias_);
    
    key_ = 1;
    systemInitialized_ = true;

    return true;
}

bool ImuOdometryOptimizer::restartOptimize()
{
    // get updated noise before reset
    gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(key_-1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(key_-1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(key_-1)));

    // reset graph
    resetGraph();

    // add pose
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
    graphFactors_.add(priorPose);

    // add velocity
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
    graphFactors_.add(priorVel);

    // add bias
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
    graphFactors_.add(priorBias);

    // add values
    graphValues_.insert(X(0), prevPose_);
    graphValues_.insert(V(0), prevVel_);
    graphValues_.insert(B(0), prevBias_);

    // optimize once
    optimizer_.update(graphFactors_, graphValues_);
    graphFactors_.resize(0);
    graphValues_.clear();

    key_ = 1;

    return true;
}

bool ImuOdometryOptimizer::processOptimize(const gtsam::Pose3& lidarPose, const std::vector<ImuSample>& imuSamples, bool degenerate)
{
    for (const ImuSample& imuSample : imuSamples)
    {
        // pop and integrate imu data that is between two optimizations
        double dt = (lastImuTime_ < 0) ? (1.0 / params_.imuRate) : (imuSample.timestamp_ - lastImuTime_);
        imuIntegrator_->integrateMeasurement(
                gtsam::Vector3(imuSample.linearAcceleration_.x(), imuSample.linearAcceleration_.y(), imuSample.linearAcceleration_.z()),
                gtsam::Vector3(imuSample.angularVelocity_.x(), imuSample.angularVelocity_.y(), imuSample.angularVelocity_.z()), dt);
        lastImuTime_ = imuSample.timestamp_;
    }

    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegrator_);
    gtsam::ImuFactor imu_factor(X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preint_imu);
    graphFactors_.add(imu_factor);

    // add imu bias between factor
    graphFactors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
                      gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegrator_->deltaTij()) * noiseModelBetweenBias_)));

    // add pose factor
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu_);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key_), curPose, degenerate ? correctionNoise2_ : correctionNoise_);
    graphFactors_.add(pose_factor);

    // insert predicted values
    gtsam::NavState propState_ = imuIntegrator_->predict(prevState_, prevBias_);
    graphValues_.insert(X(key_), propState_.pose());
    graphValues_.insert(V(key_), propState_.v());
    graphValues_.insert(B(key_), prevBias_);

    // optimize
    optimizer_.update(graphFactors_, graphValues_);
    optimizer_.update();
    graphFactors_.resize(0);
    graphValues_.clear();

    // Overwrite the beginning of the preintegration for the next step.
    gtsam::Values result = optimizer_.calculateEstimate();
    prevPose_  = result.at<gtsam::Pose3>(X(key_));
    prevVel_   = result.at<gtsam::Vector3>(V(key_));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key_));

    // Reset the optimization preintegration object.
    imuIntegrator_->resetIntegrationAndSetBias(prevBias_);

    // check optimization
    if (failureDetection(prevVel_, prevBias_))
    {
        resetStatus();
        return false;
    }

    ++key_;
    doneFirstOpt_ = true;

    return true;
}

bool ImuOdometryOptimizer::failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
{
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30)
    {
        ROS_WARN("Large velocity, reset IMU-preintegration!");
        return true;
    }

    Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0)
    {
        ROS_WARN("Large bias, reset IMU-preintegration!");
        return true;
    }

    return false;
}


