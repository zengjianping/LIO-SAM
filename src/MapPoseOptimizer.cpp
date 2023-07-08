
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
    isam = new gtsam::ISAM2(parameters);
}

MapPoseOptimizer::~MapPoseOptimizer()
{
    delete isam;
}

void MapPoseOptimizer::addOdomFactor()
{
    int numPoses = (int)mapPoseFrames->size();
    if (numPoses == 0) {
    }
    else if (numPoses == 1) {
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtsam::Pose3 poseCurr = pclPointTogtsamPose3((*mapPoseFrames)[0].pose6D);
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseCurr, priorNoise));
        initialEstimate.insert(0, poseCurr);
    }
    else {
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseCurr = pclPointTogtsamPose3((*mapPoseFrames)[numPoses-1].pose6D);
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3((*mapPoseFrames)[numPoses-2].pose6D);
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(numPoses-2, numPoses-1, poseFrom.between(poseCurr), odometryNoise));
        initialEstimate.insert(numPoses-1, poseCurr);
    }
}

void MapPoseOptimizer::addGPSFactor(std::vector<PoseSample>& gpsSamples)
{
    if (gpsSamples.empty())
        return;

    // wait for system initialized and settles down
    int numPoses = (int)mapPoseFrames->size();
    if (numPoses <= 1) {
        return;
    } else {
        if (pointDistance(mapPoseFrames->front().pose6D, mapPoseFrames->back().pose6D) < 5.0)
            return;
    }

    // pose covariance small, no need to correct
    if (poseCovariance(3,3) < options_.poseCovThreshold && poseCovariance(4,4) < options_.poseCovThreshold)
        return;

    for (const PoseSample& thisGPS : gpsSamples) {
        if (thisGPS.timestamp_ < laserCloudTime - 0.2) {
            // message too old
        } else if (thisGPS.timestamp_ > laserCloudTime + 0.2) {
            // message too new
            break;
        }  else {
            // GPS too noisy, skip
            float noise_x = thisGPS.covariance_[0];
            float noise_y = thisGPS.covariance_[7];
            float noise_z = thisGPS.covariance_[14];
            if (noise_x > options_.gpsCovThreshold || noise_y > options_.gpsCovThreshold)
                continue;

            float gps_x = thisGPS.position_.x();
            float gps_y = thisGPS.position_.y();
            float gps_z = thisGPS.position_.z();
            if (!options_.useGpsElevation) {
                gps_z = mapPoseFrames->back().pose6D.z;
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
            if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                continue;
            else
                lastGPSPoint = curGPSPoint;

            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(mapPoseFrames->size()-1, gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);

            aLoopIsClosed = true;
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
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    aLoopIsClosed = true;
}

bool MapPoseOptimizer::process(double _laserCloudTime, MapPoseFrameVecPtr& _mapPoseFrames, LoopClosureItemVecPtr& loopClosureItems,
        std::vector<PoseSample>& gpsSamples)
{
    laserCloudTime = _laserCloudTime;
    mapPoseFrames = _mapPoseFrames;
    aLoopIsClosed = false;

    // odom factor
    addOdomFactor();

    // gps factor
    addGPSFactor(gpsSamples);

    // loop factor
    addLoopFactor(loopClosureItems);

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed) {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    gtsam::Values isamCurrentEstimate = isam->calculateEstimate();
    gtsam::Pose3 latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    PointType thisPose3D;
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = mapPoseFrames->size() - 1; // this can be used as index
    mapPoseFrames->back().pose3D = thisPose3D;

    PointTypePose thisPose6D;
    thisPose6D.x = latestEstimate.translation().x();
    thisPose6D.y = latestEstimate.translation().y();
    thisPose6D.z = latestEstimate.translation().z();
    thisPose6D.intensity = mapPoseFrames->size() - 1; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = laserCloudTime;
    mapPoseFrames->back().pose6D = thisPose6D;

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    if (aLoopIsClosed) {
        int numPoses = isamCurrentEstimate.size();

        for (int i = 0; i < numPoses; ++i) {
            const gtsam::Pose3& estimate = isamCurrentEstimate.at<gtsam::Pose3>(i);

            PointType& pose3D = (*mapPoseFrames)[i].pose3D;
            pose3D.x = estimate.translation().x();
            pose3D.y = estimate.translation().y();
            pose3D.z = estimate.translation().z();

            PointTypePose& pose6D = (*mapPoseFrames)[i].pose6D;
            pose6D.x = estimate.translation().x();
            pose6D.y = estimate.translation().y();
            pose6D.z = estimate.translation().z();
            pose6D.roll = estimate.rotation().roll();
            pose6D.pitch = estimate.rotation().pitch();
            pose6D.yaw = estimate.rotation().yaw();
        }
    }
}


