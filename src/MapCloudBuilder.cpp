
#include "MapCloudBuilder.hpp"


using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose


MapCloudBuilder::MapCloudBuilder(const SystemParameter& params)
{
    params_ = params;
    float mappingCornerLeafSize = params_.mappingCornerLeafSize;
    float mappingSurfLeafSize = params_.mappingSurfLeafSize;
    float surroundingKeyframeDensity = params_.surroundingKeyframeDensity;
    float surroundingKeyframeMapLeafSize = params_.surroundingKeyframeMapLeafSize;

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    //downSizeFilterICP.setLeafSize(loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize, loopClosureICPSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization
    downSizeFilterLocalMapSurf.setLeafSize(surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize, surroundingKeyframeMapLeafSize);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    allocateMemory();
}

void MapCloudBuilder::allocateMemory()
{
    int N_SCAN = params_.N_SCAN, Horizon_SCAN = params_.Horizon_SCAN;

    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());
    laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);
    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

    for (int i = 0; i < 6; ++i) {
        transformTobeMapped[i] = 0;
    }
}

bool MapCloudBuilder::saveCloudMap(const string& dataDir, float mapResolution)
{
    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;

    string saveMapDirectory;
    if (dataDir.empty())
        saveMapDirectory = std::getenv("HOME") + params_.savePCDDirectory;
    else
        saveMapDirectory = std::getenv("HOME") + dataDir;
    cout << "Save destination: " << saveMapDirectory << endl;

    // create directory and remove old files;
    int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
    unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());

    // save key frame transformations
    pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
    pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);

    // extract global point cloud map
    pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
        *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
        *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }

    if (mapResolution != 0) {
        cout << "\n\nSave resolution: " << mapResolution << endl;

        // down-sample and save corner cloud
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        downSizeFilterCorner.setLeafSize(mapResolution, mapResolution, mapResolution);
        downSizeFilterCorner.filter(*globalCornerCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);

        // down-sample and save surf cloud
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(mapResolution, mapResolution, mapResolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
    }
    else {
        // save corner cloud
        pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
        // save surf cloud
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
    }

    // save global point cloud map
    *globalMapCloud += *globalCornerCloud;
    *globalMapCloud += *globalSurfCloud;

    int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
    bool success = ret == 0;

    float mappingCornerLeafSize = params_.mappingCornerLeafSize;
    float mappingSurfLeafSize = params_.mappingSurfLeafSize;
    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;

    return success;
}

void MapCloudBuilder::performLoopClosure(std::pair<double,double>* loopInfo)
{
    performRSLoopClosure(loopInfo);
    if (params_.enableScanContextLoopClosure)
        performSCLoopClosure();
}

void MapCloudBuilder::performRSLoopClosure(std::pair<double,double>* loopInfo)
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    mtxCloud.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtxCloud.unlock();

    // find keys
    int loopKeyCur;
    int loopKeyPre;
    if (detectLoopClosureExternal(loopInfo, &loopKeyCur, &loopKeyPre) == false)
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, params_.historyKeyframeSearchNum);
    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        return;
    //if (pubHistoryKeyFrames.getNumSubscribers() != 0) {
    //    publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    //}

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(params_.historyKeyframeSearchRadius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > params_.historyKeyframeFitnessScore)
        return;

    // publish corrected cloud
    //if (pubIcpKeyFrames.getNumSubscribers() != 0) {
    //    pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    //    pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
    //    publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    //}

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    mtxCloud.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtxCloud.unlock();

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
}

// copy from sc-lio-sam
void MapCloudBuilder::performSCLoopClosure()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    mtxCloud.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtxCloud.unlock();

    // find keys
    // first: nn index, second: yaw diff 
    auto detectResult = scManager.detectLoopClosureID(); 
    int loopKeyCur    = copy_cloudKeyPoses3D->size() - 1;;
    int loopKeyPre    = detectResult.first;
    float yawDiffRad  = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
    if( loopKeyPre == -1)
        return;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return;

    // std::cout << "SC loop found! between " << loopKeyCur << " and " << loopKeyPre << "." << std::endl; // giseop

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    int base_key = 0;
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, params_.historyKeyframeSearchNum, base_key); // giseop 
    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        return;
    //if (pubHistoryKeyFrames.getNumSubscribers() != 0) {
    //    publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    //}

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(params_.historyKeyframeSearchRadius*2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > params_.historyKeyframeFitnessScore)
        return;

    // publish corrected cloud
    //if (pubIcpKeyFrames.getNumSubscribers() != 0)
    //{
    //    pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    //    pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
    //    publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    //}

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // // transform from world origin to wrong pose
    // Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // // transform from world origin to corrected pose
    // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    // pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
    // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    // gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

    // gtsam::Vector Vector6(6);
    // float noiseScore = icp.getFitnessScore();
    // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    // noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    // giseop 
    pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    // giseop, robust kernel for a SC loop
    float robustNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); 
    robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
    noiseModel::Base::shared_ptr robustConstraintNoise; 
    robustConstraintNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
    ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

    // Add pose constraint
    mtxCloud.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(robustConstraintNoise);
    mtxCloud.unlock();

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
}

bool MapCloudBuilder::detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
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
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), params_.historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > params_.historyKeyframeSearchTimeDiff) {
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

bool MapCloudBuilder::detectLoopClosureExternal(std::pair<double,double>* loopInfo, int *latestID, int *closestID)
{
    // this function is not used yet, please ignore it
    int loopKeyCur = -1;
    int loopKeyPre = -1;

    if (!loopInfo)
        return false;
    double loopTimeCur = loopInfo->first;
    double loopTimePre = loopInfo->second;
    if (abs(loopTimeCur - loopTimePre) < params_.historyKeyframeSearchTimeDiff)
        return false;

    int cloudSize = copy_cloudKeyPoses6D->size();
    if (cloudSize < 2)
        return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i) {
        if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
            loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i) {
        if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
            loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
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

void MapCloudBuilder::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum, const int loop_index)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        int select_loop_index = (loop_index != -1) ? loop_index : keyNear;
        *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[select_loop_index]);
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[select_loop_index]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

bool MapCloudBuilder::processLaserCloud(const PointCloudInfo& _cloudInfo, const std::vector<PoseSample>& gpsSamples)
{
    std::lock_guard<std::mutex> lock(mtxCloud);

    cloudInfo = _cloudInfo;
    timeLaserInfoCur = cloudInfo.timestamp;
    laserCloudCornerLast = cloudInfo.cornerCloud;
    laserCloudSurfLast = cloudInfo.surfaceCloud;

    if (timeLaserInfoCur - timeLastProcessing >= params_.mappingProcessInterval) {
        timeLastProcessing = timeLaserInfoCur;

        updateInitialGuess();

        extractSurroundingKeyFrames();

        downsampleCurrentScan();

        scan2MapOptimization();

        saveKeyFramesAndFactor(gpsSamples);

        correctPoses();

        return true;
    }
    return false;
}

void MapCloudBuilder::updateInitialGuess()
{
    // save current transformation before any processing
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    // initialization
    if (cloudKeyPoses3D->points.empty()) {
        transformTobeMapped[0] = cloudInfo.imuRollInit;
        transformTobeMapped[1] = cloudInfo.imuPitchInit;
        transformTobeMapped[2] = cloudInfo.imuYawInit;

        if (!params_.useImuHeadingInitialization)
            transformTobeMapped[2] = 0;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }
    else if (0) {
        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * incrementalOdometryAffineIncr;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        return;
    }

    // use imu pre-integration estimation for pose guess
    if (cloudInfo.odomAvailable == true) {
        Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                            cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
        if (lastImuPreTransAvailable == false) {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        }
        else {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            lastImuPreTransformation = transBack;
            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    // use imu incremental estimation for pose guess (only rotation)
    if (cloudInfo.imuAvailable == true && params_.imuType == 0) {
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }
}

void MapCloudBuilder::extractForLoopClosure()
{
    pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses-1; i >= 0; --i) {
        if ((int)cloudToExtract->size() <= params_.surroundingKeyframeSize)
            cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    extractCloud(cloudToExtract);
}

void MapCloudBuilder::extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses; // 在构建局部地图时挑选的周围关键帧的三维姿态（构建kdtree加速搜索）
    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)params_.surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
    for (int i = 0; i < (int)pointSearchInd.size(); ++i) {
        int id = pointSearchInd[i];
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }

    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    for(auto& pt : surroundingKeyPosesDS->points) {
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }

    // also extract some latest key frames in case the robot rotates in one position
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses-1; i >= 0; --i) {
        if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    extractCloud(surroundingKeyPosesDS);
}

void MapCloudBuilder::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear(); 
    for (int i = 0; i < (int)cloudToExtract->size(); ++i) {
        if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > params_.surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
            // transformed cloud available
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        }
        else {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap   += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
        
    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

    // clear map cache if too large
    if (laserCloudMapContainer.size() > 1000)
        laserCloudMapContainer.clear();
}

void MapCloudBuilder::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return; 
    
    // if (loopClosureEnableFlag == true)
    // {
    //     extractForLoopClosure();    
    // } else {
    //     extractNearby();
    // }

    extractNearby();
}

void MapCloudBuilder::downsampleCurrentScan()
{
    // Downsample cloud from current scan
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

void MapCloudBuilder::scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (laserCloudCornerLastDSNum > params_.edgeFeatureMinValidNum && laserCloudSurfLastDSNum > params_.surfFeatureMinValidNum)
    {
        laserCloudRegister_->setEdgeFeatureCloud(laserCloudCornerLastDS, laserCloudCornerFromMapDS);
        laserCloudRegister_->setSurfFeatureCloud(laserCloudSurfLastDS, laserCloudSurfFromMapDS);
        laserCloudRegister_->process(currInitPose, currFinalPose);
    }
    else {
        printf("Not enough features! Only %d edge and %d planar features available.\n", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
    }
}

void MapCloudBuilder::transformUpdate()
{
    /*if (cloudInfo.imuAvailable == true && params_.imuType == 0) {
        if (std::abs(cloudInfo.imuPitchInit) < 1.4) {
            double imuWeight = params_.imuRPYWeight;
            tf::Quaternion imuQuaternion;
            tf::Quaternion transformQuaternion;
            double rollMid, pitchMid, yawMid;

            // slerp roll
            transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
            imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[0] = rollMid;

            // slerp pitch
            transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
            imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
            transformTobeMapped[1] = pitchMid;
        }
    }*/

    transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], params_.rotation_tollerance);
    transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], params_.rotation_tollerance);
    transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], params_.z_tollerance);

    incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    incrementalOdometryAffineIncr = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
}

float MapCloudBuilder::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;
    return value;
}

bool MapCloudBuilder::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    if (params_.sensor == SensorType::LIVOX)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
            return true;
    }

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll)  < params_.surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < params_.surroundingkeyframeAddingAngleThreshold && 
        abs(yaw)   < params_.surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < params_.surroundingkeyframeAddingDistThreshold) {
        return false;
    }

    return true;
}

void MapCloudBuilder::addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty()) {
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    else {
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

void MapCloudBuilder::addGPSFactor(const std::vector<PoseSample>& gpsSamples)
{
    if (gpsSamples.empty())
        return;

    // wait for system initialized and settles down
    if (cloudKeyPoses3D->points.empty()) {
        return;
    }
    else {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }

    // pose covariance small, no need to correct
    if (poseCovariance(3,3) < params_.poseCovThreshold && poseCovariance(4,4) < params_.poseCovThreshold)
        return;

    for (const PoseSample& thisGPS : gpsSamples) {
        if (thisGPS.timestamp_ < timeLaserInfoCur - 0.2) {
            // message too old
        }
        else if (thisGPS.timestamp_ > timeLaserInfoCur + 0.2) {
            // message too new
            break;
        }
        else {
            // GPS too noisy, skip
            float noise_x = thisGPS.covariance_[0];
            float noise_y = thisGPS.covariance_[7];
            float noise_z = thisGPS.covariance_[14];
            if (noise_x > params_.gpsCovThreshold || noise_y > params_.gpsCovThreshold)
                continue;

            float gps_x = thisGPS.position_.x();
            float gps_y = thisGPS.position_.y();
            float gps_z = thisGPS.position_.z();
            if (!params_.useGpsElevation) {
                gps_z = transformTobeMapped[5];
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
            noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);

            aLoopIsClosed = true;
            break;
        }
    }
}

void MapCloudBuilder::addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        //gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        auto noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

void MapCloudBuilder::saveKeyFramesAndFactor(const std::vector<PoseSample>& gpsSamples)
{
    if (saveFrame() == false)
        return;

    // odom factor
    addOdomFactor();

    // gps factor
    addGPSFactor(gpsSamples);

    // loop factor
    addLoopFactor();

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
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
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
    thisPose6D.roll  = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw   = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

    // save key frame cloud
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    if (params_.enableScanContextLoopClosure) {
        // The following code is copy from sc-lio-sam
        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

        if( sc_input_type == SCInputType::SINGLE_SCAN_FULL ) {
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame = cloudInfo.extractedCloud;
            scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        }  
        else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) { 
            scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame); 
        }
        else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) { 
            pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
            loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, params_.historyKeyframeSearchNum, -1);
            scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud); 
        }
    }
}

void MapCloudBuilder::correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true) {
        // clear map cache
        laserCloudMapContainer.clear();            // update key poses
        int numPoses = isamCurrentEstimate.size();

        for (int i = 0; i < numPoses; ++i) {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
        }

        aLoopIsClosed = false;
    }
}


