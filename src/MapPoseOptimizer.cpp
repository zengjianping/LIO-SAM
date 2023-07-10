
#include "MapPoseOptimizer.hpp"


using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::G; // GPS pose


MapPoseOptimizer::MapPoseOptimizer(const Options& options)
{
    options_ = options;

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = new gtsam::ISAM2(parameters);
}

MapPoseOptimizer::~MapPoseOptimizer()
{
    delete isam_;
}

void MapPoseOptimizer::addOdomFactor()
{
    int numPoses = (int)mapPoseFrames_->size();
    if (numPoses == 0) {
    }
    else if (numPoses == 1) {
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtsam::Pose3 poseCurr = (*mapPoseFrames_)[0].pose.toGtsamPose();
        gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseCurr, priorNoise));
        initialEstimate_.insert(0, poseCurr);
    }
    else {
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseCurr = (*mapPoseFrames_)[numPoses-1].pose.toGtsamPose();
        gtsam::Pose3 poseFrom = (*mapPoseFrames_)[numPoses-2].pose.toGtsamPose();
        gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(numPoses-2, numPoses-1, poseFrom.between(poseCurr), odometryNoise));
        initialEstimate_.insert(numPoses-1, poseCurr);
    }
}

void MapPoseOptimizer::addGPSFactor(std::vector<EntityPose>& gpsSamples)
{
    if (gpsSamples.empty())
        return;

    // wait for system initialized and settles down
    int numPoses = (int)mapPoseFrames_->size();
    if (numPoses <= 1) {
        return;
    } else {
        const Eigen::Vector3d& pos1 = mapPoseFrames_->front().pose.position;
        const Eigen::Vector3d& pos2 = mapPoseFrames_->back().pose.position;
        if ((pos1 - pos2).norm() < 5.0)
            return;
    }

    // pose covariance small, no need to correct
    if (poseCovariance_(3,3) < options_.poseCovThreshold && poseCovariance_(4,4) < options_.poseCovThreshold)
        return;

    for (const EntityPose& thisGPS : gpsSamples) {
        if (thisGPS.timestamp < laserCloudTime_ - 0.2) {
            // message too old
        } else if (thisGPS.timestamp > laserCloudTime_ + 0.2) {
            // message too new
            break;
        }  else {
            // GPS too noisy, skip
            float noise_x = thisGPS.covariance[0];
            float noise_y = thisGPS.covariance[7];
            float noise_z = thisGPS.covariance[14];
            if (noise_x > options_.gpsCovThreshold || noise_y > options_.gpsCovThreshold)
                continue;

            float gps_x = thisGPS.position.x();
            float gps_y = thisGPS.position.y();
            float gps_z = thisGPS.position.z();
            if (!options_.useGpsElevation) {
                gps_z = mapPoseFrames_->back().pose.position.z();
                noise_z = 0.01;
            }

            // GPS not properly initialized (0,0,0)
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                continue;

            // Add GPS every a few meters
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (pointDistance(curGPSPoint, lastGPSPoint_) < 5.0)
                continue;
            else
                lastGPSPoint_ = curGPSPoint;

            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(mapPoseFrames_->size()-1, gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph_.add(gps_factor);

            aLoopIsClosed_ = true;
            break;
        }
    }
}

void MapPoseOptimizer::addLoopFactor(LoopClosureItemVecPtr& loopClosureItems)
{
    if (loopClosureItems->empty())
        return;

    for (int i = 0; i < (int)loopClosureItems->size(); ++i) {
        int indexFrom = (*loopClosureItems)[i].keyCur;
        int indexTo = (*loopClosureItems)[i].keyPre;
        const gtsam::Pose3& poseBetween = (*loopClosureItems)[i].pose;
        const auto& noiseBetween = (*loopClosureItems)[i].noise;
        gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    aLoopIsClosed_ = true;
}

bool MapPoseOptimizer::process(double laserCloudTime, MapPoseFrameVecPtr& mapPoseFrames, LoopClosureItemVecPtr& loopClosureItems,
        std::vector<EntityPose>& gpsSamples)
{
    laserCloudTime_ = laserCloudTime;
    mapPoseFrames_ = mapPoseFrames;
    aLoopIsClosed_ = false;

    // odom factor
    addOdomFactor();

    // gps factor
    addGPSFactor(gpsSamples);

    // loop factor
    addLoopFactor(loopClosureItems);

    // cout << "****************************************************" << endl;
    // gtSAMgraph_.print("GTSAM Graph:\n");

    // update iSAM
    isam_->update(gtSAMgraph_, initialEstimate_);
    isam_->update();

    if (aLoopIsClosed_) {
        isam_->update();
        isam_->update();
        isam_->update();
        isam_->update();
        isam_->update();
    }

    gtSAMgraph_.resize(0);
    initialEstimate_.clear();

    gtsam::Values isamCurrentEstimate = isam_->calculateEstimate();
    gtsam::Pose3 latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    /*PointTypePose thisPose6D;
    thisPose6D.x = latestEstimate.translation().x();
    thisPose6D.y = latestEstimate.translation().y();
    thisPose6D.z = latestEstimate.translation().z();
    thisPose6D.intensity = mapPoseFrames_->size() - 1; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = laserCloudTime_;
    mapPoseFrames_->back().pose6D = thisPose6D;*/
    mapPoseFrames_->back().pose = EntityPose(latestEstimate);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam_->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance_ = isam_->marginalCovariance(isamCurrentEstimate.size()-1);

    if (aLoopIsClosed_) {
        int numPoses = isamCurrentEstimate.size();

        for (int i = 0; i < numPoses; ++i) {
            const gtsam::Pose3& estimate = isamCurrentEstimate.at<gtsam::Pose3>(i);

            /*PointTypePose& pose6D = (*mapPoseFrames_)[i].pose6D;
            pose6D.x = estimate.translation().x();
            pose6D.y = estimate.translation().y();
            pose6D.z = estimate.translation().z();
            pose6D.roll = estimate.rotation().roll();
            pose6D.pitch = estimate.rotation().pitch();
            pose6D.yaw = estimate.rotation().yaw();*/
            (*mapPoseFrames_)[i].pose = EntityPose(estimate);
        }
    }

    return true;
}


