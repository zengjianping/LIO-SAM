
#include "LaserLoopDetector.hpp"


LaserLoopDetector::LaserLoopDetector(const Options& options)
{
    options_ = options;

    float mappingIcpLeafSize = options_.mappingIcpLeafSize;
    downSizeFilterICP.setLeafSize(mappingIcpLeafSize, mappingIcpLeafSize, mappingIcpLeafSize);
}

bool LaserLoopDetector::process(const pcl::PointCloud<PointType>::Ptr& _cloudKeyPoses3D, const pcl::PointCloud<PointTypePose>::Ptr& _cloudKeyPoses6D,
        const vector<pcl::PointCloud<PointType>::Ptr>& _cornerCloudKeyFrames, const vector<pcl::PointCloud<PointType>::Ptr>& _surfCloudKeyFrames,
        vector<pair<int, int>>& _loopIndexQueue, vector<gtsam::Pose3>& _loopPoseQueue, vector<gtsam::SharedNoiseModel>& _loopNoiseQueue,
        double laserTime, std::pair<double,double>* loopInfo)
{
    timeLaserInfoCur = laserTime;
    cloudKeyPoses3D = _cloudKeyPoses3D;
    cloudKeyPoses6D = _cloudKeyPoses6D;
    cornerCloudKeyFrames = _cornerCloudKeyFrames;
    surfCloudKeyFrames = _surfCloudKeyFrames;

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();

    performRSLoopClosure(loopInfo);
    if (options_.enableScanContextLoopClosure)
        performSCLoopClosure();

    _loopIndexQueue = loopIndexQueue;
    _loopPoseQueue = loopPoseQueue;
    _loopNoiseQueue = loopNoiseQueue;
    
    return true;
}

bool LaserLoopDetector::performRSLoopClosure(std::pair<double,double>* loopInfo)
{
    if (cloudKeyPoses3D->points.empty() == true)
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
    Eigen::Affine3f tWrong = pclPointToAffine3f(cloudKeyPoses6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;

    return true;
}

// copy from sc-lio-sam
bool LaserLoopDetector::performSCLoopClosure()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return false;

    // find keys
    // first: nn index, second: yaw diff 
    auto detectResult = scManager.detectLoopClosureID(); 
    int loopKeyCur    = cloudKeyPoses3D->size() - 1;;
    int loopKeyPre    = detectResult.first;
    float yawDiffRad  = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
    if( loopKeyPre == -1)
        return false;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
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
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(robustConstraintNoise);

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;

    return true;
}

bool LaserLoopDetector::detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    // find the closest history key frame
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses; // 在构建局部地图时挑选的邻近时间关键帧的三维姿态（构建kdtree加速搜索）
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(cloudKeyPoses3D->back(), options_.historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (abs(cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > options_.historyKeyframeSearchTimeDiff) {
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

    int cloudSize = cloudKeyPoses6D->size();
    if (cloudSize < 2)
        return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i) {
        if (cloudKeyPoses6D->points[i].time >= loopTimeCur)
            loopKeyCur = round(cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i) {
        if (cloudKeyPoses6D->points[i].time <= loopTimePre)
            loopKeyPre = round(cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    if (loopKeyCur == loopKeyPre)
        return false;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

void LaserLoopDetector::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int loop_index)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        int select_loop_index = (loop_index != -1) ? loop_index : keyNear;
        *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &cloudKeyPoses6D->points[select_loop_index]);
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &cloudKeyPoses6D->points[select_loop_index]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

