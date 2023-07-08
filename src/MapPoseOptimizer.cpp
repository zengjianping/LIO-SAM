
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

inline gtsam::Pose3 poseEigen2Gtsam(const Eigen::Isometry3d& eigenPose)
{
    return gtsam::Pose3(eigenPose.matrix());
}

void MapPoseOptimizer::addOdomFactor(const Eigen::Isometry3d& odomPose)
{
    gtsam::Pose3 poseCurr = poseEigen2Gtsam(odomPose);

    if (cloudKeyPoses3D->points.empty()) {
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseCurr, priorNoise));
        initialEstimate.insert(0, poseCurr);
    }  else {
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseCurr), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseCurr);
    }
}

void MapPoseOptimizer::addGPSFactor(std::vector<PoseSample>& gpsSamples)
{
    if (gpsSamples.empty())
        return;

    // wait for system initialized and settles down
    if (cloudKeyPoses3D->points.empty()) {
        return;
    } else {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }

    // pose covariance small, no need to correct
    if (poseCovariance(3,3) < options_.poseCovThreshold && poseCovariance(4,4) < options_.poseCovThreshold)
        return;

    for (const PoseSample& thisGPS : gpsSamples) {
        if (thisGPS.timestamp_ < timeLaserInfoCur - 0.2) {
            // message too old
        } else if (thisGPS.timestamp_ > timeLaserInfoCur + 0.2) {
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
                gps_z = lastPose.translation().z();
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
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);

            aLoopIsClosed = true;
            break;
        }
    }
}

void MapPoseOptimizer::addLoopFactor(vector<pair<int, int>>& loopIndexQueue, vector<gtsam::Pose3>& loopPoseQueue,
        vector<gtsam::SharedNoiseModel>& loopNoiseQueue)
{
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        //gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        auto noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

bool MapPoseOptimizer::process(pcl::PointCloud<PointType>::Ptr& _cloudKeyPoses3D, pcl::PointCloud<PointTypePose>::Ptr& _cloudKeyPoses6D,
        double laserTime, Eigen::Isometry3d& odomPose, std::vector<PoseSample>& gpsSamples,
        vector<pair<int, int>>& loopIndexQueue, vector<gtsam::Pose3>& loopPoseQueue, vector<gtsam::SharedNoiseModel>& loopNoiseQueue)
{
    timeLaserInfoCur = laserTime;
    lastPose = odomPose;
    cloudKeyPoses3D = _cloudKeyPoses3D;
    cloudKeyPoses6D = _cloudKeyPoses6D;

    aLoopIsClosed = false;

    // odom factor
    addOdomFactor(odomPose);

    // gps factor
    addGPSFactor(gpsSamples);

    // loop factor
    addLoopFactor(loopIndexQueue, loopPoseQueue, loopNoiseQueue);

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

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    gtsam::Values isamCurrentEstimate; // 优化器当前优化结果
    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // save updated transform
    odomPose = Eigen::Isometry3d(latestEstimate.matrix());

    if (aLoopIsClosed) {
        int numPoses = isamCurrentEstimate.size();

        for (int i = 0; i < numPoses; ++i) {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
        }
    }
}


