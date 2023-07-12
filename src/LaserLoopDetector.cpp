
#include "LaserLoopDetector.hpp"


LaserLoopDetector::LaserLoopDetector(const Options& options)
{
    options_ = options;

    float loopClosureICPSurfLeafSize = options_.loopClosureICPSurfLeafSize;
    downSizeFilterICP_.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
}

bool LaserLoopDetector::process(double laserCloudTime, MapPoseFrameVecPtr& mapPoseFrames, LoopClosureItemVecPtr& loopClosureItems,
        std::pair<double,double>* loopInfo)
{
    laserCloudTime_ = laserCloudTime;
    mapPoseFrames_ = mapPoseFrames;
    loopClosureItems_ = loopClosureItems;

    loopClosureItems_->clear();

    performRSLoopClosure(loopInfo);
    if (options_.enableScanContextLoopClosure)
        performSCLoopClosure();

    return true;
}

bool LaserLoopDetector::performRSLoopClosure(std::pair<double,double>* loopInfo)
{
    if (mapPoseFrames_->empty())
        return false;

    // find keys
    int loopKeyCur, loopKeyPre;
    if (detectLoopClosureExternal(loopInfo, &loopKeyCur, &loopKeyPre) == false)
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return false;

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, options_.historyKeyframeSearchNum);
    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        return false;

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(options_.historyKeyframeSearchRadius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > options_.historyKeyframeFitnessScore)
        return false;

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    //Eigen::Affine3f tWrong = pclPointToAffine3f((*mapPoseFrames_)[loopKeyCur].pose6D);
    Eigen::Affine3f tWrong = (*mapPoseFrames_)[loopKeyCur].pose.toAffine().cast<float>();
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    //gtsam::Pose3 poseTo = pclPointTogtsamPose3((*mapPoseFrames_)[loopKeyPre].pose6D);
    gtsam::Pose3 poseTo = (*mapPoseFrames_)[loopKeyPre].pose.toGtsamPose();
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    LoopClosureItem loopClosureItem;
    loopClosureItem.keyCur = loopKeyCur;
    loopClosureItem.keyPre = loopKeyPre;
    loopClosureItem.pose = poseFrom.between(poseTo);
    loopClosureItem.noise = constraintNoise;
    loopClosureItems_->push_back(loopClosureItem);

    // add loop constriant
    loopIndexContainer_[loopKeyCur] = loopKeyPre;

    return true;
}

// copy from sc-lio-sam
bool LaserLoopDetector::performSCLoopClosure()
{
    if (mapPoseFrames_->empty())
        return false;

    pcl::PointCloud<PointType>::Ptr scCloud;
    // The following code is copy from sc-lio-sam
    // Scan Context loop detector - giseop
    // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
    // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
    // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
    const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

    if( sc_input_type == SCInputType::SINGLE_SCAN_FULL ) {
        scCloud = mapPoseFrames_->back().extractedCloud;
    } 
    else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) {
        scCloud = mapPoseFrames_->back().surfCloud;
    }
    else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) { 
        pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
        loopFindNearKeyframes(multiKeyFrameFeatureCloud, mapPoseFrames_->size() - 1, options_.historyKeyframeSearchNum, -1);
        scCloud = multiKeyFrameFeatureCloud;
    }
    else {
        return false;
    }
    scManager_.makeAndSaveScancontextAndKeys(*scCloud);

    // find keys
    // first: nn index, second: yaw diff 
    auto detectResult = scManager_.detectLoopClosureID(); 
    int loopKeyCur = mapPoseFrames_->size() - 1;;
    int loopKeyPre = detectResult.first;
    //sfloat yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
    if( loopKeyPre == -1)
        return false;

    auto it = loopIndexContainer_.find(loopKeyCur);
    if (it != loopIndexContainer_.end())
        return false;

    // std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    int base_key = 0;
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, options_.historyKeyframeSearchNum, base_key); // giseop 
    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        return false;
    //if (pubHistoryKeyFrames.getNumSubscribers() != 0) {
    //    publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    //}

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(options_.historyKeyframeSearchRadius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > options_.historyKeyframeFitnessScore)
        return false;

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // // transform from world origin to wrong pose
    // Eigen::Affine3f tWrong = pclPointToAffine3f(cloudKeyPoses6D->points[loopKeyCur]);
    // // transform from world origin to corrected pose
    // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    // pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    // gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[loopKeyPre]);

    // gtsam::Vector Vector6(6);
    // float noiseScore = icp.getFitnessScore();
    // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    // noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    // giseop 
    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

    // giseop, robust kernel for a SC loop
    float robustNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); 
    robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
    gtsam::noiseModel::Base::shared_ptr robustConstraintNoise; 
    robustConstraintNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
    ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

    // Add pose constraint
    LoopClosureItem loopClosureItem;
    loopClosureItem.keyCur = loopKeyCur;
    loopClosureItem.keyPre = loopKeyPre;
    loopClosureItem.pose = poseFrom.between(poseTo);
    loopClosureItem.noise = robustConstraintNoise;
    loopClosureItems_->push_back(loopClosureItem);

    // add loop constriant
    loopIndexContainer_[loopKeyCur] = loopKeyPre;

    return true;
}

bool LaserLoopDetector::detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = mapPoseFrames_->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loopIndexContainer_.find(loopKeyCur);
    if (it != loopIndexContainer_.end())
        return false;

    // find the closest history key frame
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses; // 在构建局部地图时挑选的邻近时间关键帧的三维姿态（构建kdtree加速搜索）
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    for (const MapPoseFrame& frame : *mapPoseFrames_) {
        cloudKeyPoses3D->points.push_back(pose3DFromPose(frame.pose));
    }
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(cloudKeyPoses3D->back(), options_.historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (abs((*mapPoseFrames_)[id].pose.timestamp - laserCloudTime_) > options_.historyKeyframeSearchTimeDiff) {
            loopKeyPre = id;
            break;
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

bool LaserLoopDetector::detectLoopClosureExternal(std::pair<double,double>* loopInfo, int *latestID, int *closestID)
{
    // this function is not used yet, please ignore it
    int loopKeyCur = -1;
    int loopKeyPre = -1;

    if (!loopInfo)
        return false;
    double loopTimeCur = loopInfo->first;
    double loopTimePre = loopInfo->second;
    if (abs(loopTimeCur - loopTimePre) < options_.historyKeyframeSearchTimeDiff)
        return false;

    int cloudSize = mapPoseFrames_->size();
    if (cloudSize < 2)
        return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i) {
        if ((*mapPoseFrames_)[i].pose.timestamp >= loopTimeCur)
            loopKeyCur = round((*mapPoseFrames_)[i].pose.index);
        else
            break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i) {
        if ((*mapPoseFrames_)[i].pose.timestamp <= loopTimePre)
            loopKeyPre = round((*mapPoseFrames_)[i].pose.index);
        else
            break;
    }

    if (loopKeyCur == loopKeyPre)
        return false;

    auto it = loopIndexContainer_.find(loopKeyCur);
    if (it != loopIndexContainer_.end())
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

void LaserLoopDetector::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int loop_index)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = mapPoseFrames_->size();
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        int select_loop_index = (loop_index != -1) ? loop_index : keyNear;
        const EntityPose& pose = (*mapPoseFrames_)[select_loop_index].pose;
        *nearKeyframes += *transformPointCloud((*mapPoseFrames_)[keyNear].cornerCloud, pose);
        *nearKeyframes += *transformPointCloud((*mapPoseFrames_)[keyNear].surfCloud, pose);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP_.setInputCloud(nearKeyframes);
    downSizeFilterICP_.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}


