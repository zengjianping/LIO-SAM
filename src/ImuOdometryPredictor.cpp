
#include "ImuOdometryPredictor.hpp"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)


ImuOdometryPredictor::ImuOdometryPredictor(const Options& options)
{
    options_ = options;

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(options_.imuGravity);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(options_.imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(options_.imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias priorImuBias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
    imuIntegratorOpt_.reset(new gtsam::PreintegratedImuMeasurements(p, priorImuBias)); // setting up the IMU integration for optimization
    imuIntegratorPrd_.reset(new gtsam::PreintegratedImuMeasurements(p, priorImuBias)); // setting up the IMU integration for IMU message thread

    priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise_  = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
    priorBiasNoise_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    correctionNoise2_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m

    float imuAccBiasN = options_.imuAccBiasN, imuGyrBiasN = options_.imuGyrBiasN;
    noiseModelBetweenBias_ = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

    resetStatus();
}

//int pncnt = 3;

bool ImuOdometryPredictor::predict(const EntityPose& imuSample, EntityPose& imuState)
{
    imuQueue_.push_back(imuSample);

    if (!systemInitialized_) {
        return false;
    }

    // predict odometry
    gtsam::NavState gtsamState = predictOdometry(odomState_, odomBias_, imuSample);
    imuState = EntityPose(gtsamState);
    imuState.timestamp = imuSample.timestamp;
    imuState.index = imuSample.index;
    //if (pncnt > 0) {
    //    //cout << "Imu sample: " << imuSample.print() << endl;
    //    cout << "Imu state: " << imuState.print() << endl;
    //    pncnt--;
    //}

    return true;
}

bool ImuOdometryPredictor::reset(const EntityPose& odomState, bool degenerate)
{
    const double delta_t = 0; // 在做IMU数据和雷达里程计同步过程中的时间间隔
    gtsam::NavState gtsamState = odomState.toGtsamState();
    double odomTime = odomState.timestamp;
    //cout << "Odom state: " << odomState.print() << endl;

    // 1. integrate imu data and optimize
    std::vector<EntityPose> imuSamples;
    double lastImuQT = -1;
    while (!imuQueue_.empty()) {
        // pop and integrate imu data that is between two optimizations
        const EntityPose& imuSample = imuQueue_.front();
        if (imuSample.timestamp < odomTime - delta_t) {
            imuSamples.push_back(imuSample);
            lastImuQT = imuSample.timestamp;
            imuQueue_.pop_front();
        } else {
            break;
        }
    }
    if (!optimizeOdometry(gtsamState, imuSamples, degenerate)) {
        return false;
    }

    // 2. after optiization, re-propagate imu odometry preintegration
    // first pop imu message older than current correction data
    // integrate imu message from the beginning of this optimization
    imuSamples.clear();
    for (int i = 0; i < (int)imuQueue_.size(); ++i) {
        EntityPose imuSample = imuQueue_[i];
        imuSamples.push_back(imuSample);
    }
    resetImuIntegrator(odomBias_, imuSamples, lastImuQT);

    //pncnt = 3;
    return true;
}

gtsam::NavState ImuOdometryPredictor::predictOdometry(const gtsam::NavState& prevState, const gtsam::imuBias::ConstantBias& prevBias, const EntityPose& imuSample)
{
    double dt = (lastImuTimePrd_ < 0) ? (1.0 / options_.imuRate) : (imuSample.timestamp - lastImuTimePrd_);
    lastImuTimePrd_ = imuSample.timestamp;
    //cout << "predict dt: " << dt << ", index: " << imuSample.index << ", time: " << imuSample.timestamp << endl;

    // integrate this single imu message
    imuIntegratorPrd_->integrateMeasurement(
        gtsam::Vector3(imuSample.linearAcc.x(), imuSample.linearAcc.y(), imuSample.linearAcc.z()),
        gtsam::Vector3(imuSample.angularVel.x(), imuSample.angularVel.y(), imuSample.angularVel.z()), dt);

    // predict odometry
    gtsam::NavState imuState = imuIntegratorPrd_->predict(prevState, prevBias);
    return imuState;
}

bool ImuOdometryPredictor::resetImuIntegrator(const gtsam::imuBias::ConstantBias& prevBias, const std::vector<EntityPose>& imuSamples, double lastImuTime)
{
    // reset bias use the newly optimized bias
    imuIntegratorPrd_->resetIntegrationAndSetBias(prevBias);
    lastImuTimePrd_ = lastImuTime;
    //cout << "lastImuTimePrd: " << lastImuTimePrd_ << endl;

    // integrate imu message from the beginning of this optimization
    for (const EntityPose& imuSample : imuSamples) {
        // pop and integrate imu data that is between two optimizations
        double dt = (lastImuTimePrd_ < 0) ? (1.0 / options_.imuRate) : (imuSample.timestamp - lastImuTimePrd_);
        //cout << "reset dt: " << dt << ", index: " << imuSample.index << ", time: " << imuSample.timestamp << endl;
        imuIntegratorPrd_->integrateMeasurement(
            gtsam::Vector3(imuSample.linearAcc.x(), imuSample.linearAcc.y(), imuSample.linearAcc.z()),
            gtsam::Vector3(imuSample.angularVel.x(), imuSample.angularVel.y(), imuSample.angularVel.z()), dt);
        lastImuTimePrd_ = imuSample.timestamp;
    }

    return true;
}

void ImuOdometryPredictor::resetGraph()
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

void ImuOdometryPredictor::resetStatus()
{
    systemInitialized_ = false;
    doneFirstOpt_ = false;
    lastImuTimeOpt_ = -1;
    lastImuTimePrd_ = -1;
    key_ = 0;
}

bool ImuOdometryPredictor::optimizeOdometry(const gtsam::NavState& odomState, const std::vector<EntityPose>& imuSamples, bool degenerate)
{
    if (!systemInitialized_) {
        // initialize system
        return startOptimize(odomState, imuSamples);
    }
    
    if (key_ == 100) {
        // reset graph for speed
        restartOptimize();
    }

    // integrate imu data and optimize
    return processOptimize(odomState, imuSamples, degenerate);
}

bool ImuOdometryPredictor::startOptimize(const gtsam::NavState& odomState, const std::vector<EntityPose>& imuSamples)
{
    for (const EntityPose& imuSample : imuSamples) {
        lastImuTimeOpt_ = imuSample.timestamp;
    }

    // reset graph
    resetGraph();

    // initial pose
    prevPose_ = odomState.pose();
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise_);
    graphFactors_.add(priorPose);

    // initial velocity
    prevVel_ = odomState.velocity(); //gtsam::Vector3(0, 0, 0);
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

    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

    prevState_ = gtsam::NavState(prevPose_, prevVel_); //odomState;
    odomState_ = prevState_;
    odomBias_  = prevBias_;
    
    key_ = 1;
    systemInitialized_ = true;

    return true;
}

bool ImuOdometryPredictor::restartOptimize()
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

bool ImuOdometryPredictor::processOptimize(const gtsam::NavState& odomState, const std::vector<EntityPose>& imuSamples, bool degenerate)
{
    //cout << "lastImuTimeOpt: " << lastImuTimeOpt_ << endl;
    for (const EntityPose& imuSample : imuSamples) {
        // pop and integrate imu data that is between two optimizations
        double dt = (lastImuTimeOpt_ < 0) ? (1.0 / options_.imuRate) : (imuSample.timestamp - lastImuTimeOpt_);
        //cout << "optimize dt: " << dt << ", index: " << imuSample.index << ", time: " << imuSample.timestamp << endl;
        imuIntegratorOpt_->integrateMeasurement(
                gtsam::Vector3(imuSample.linearAcc.x(), imuSample.linearAcc.y(), imuSample.linearAcc.z()),
                gtsam::Vector3(imuSample.angularVel.x(), imuSample.angularVel.y(), imuSample.angularVel.z()), dt);
        lastImuTimeOpt_ = imuSample.timestamp;
    }

    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preintImu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    gtsam::ImuFactor imuFactor(X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preintImu);
    graphFactors_.add(imuFactor);

    // add imu bias between factor
    graphFactors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
                      gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias_)));

    // add pose factor
    gtsam::Pose3 curPose = odomState.pose();
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key_), curPose, degenerate ? correctionNoise2_ : correctionNoise_);
    graphFactors_.add(pose_factor);

    // insert predicted values
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
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
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

    // check optimization
    if (failureDetection(prevVel_, prevBias_)) {
        resetStatus();
        return false;
    }
    odomState_ = prevState_;
    odomBias_  = prevBias_;
    //cout << "Prev state: " << EntityPose(odomState_).print() << endl;

    ++key_;
    doneFirstOpt_ = true;

    return true;
}

bool ImuOdometryPredictor::failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
{
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30) {
        cout << "Large velocity, reset IMU-preintegration!" << endl;
        return true;
    }

    Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0) {
        cout << "Large bias, reset IMU-preintegration!" << endl;
        return true;
    }

    return false;
}


