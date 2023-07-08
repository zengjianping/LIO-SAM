
#include "MapCloudBuilder.hpp"


enum class SCInputType 
{ 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 

MapCloudBuilder::MapCloudBuilder(const Options& options)
{
    options_  = options;

    float mappingCornerLeafSize = options_.mappingCornerLeafSize;
    float mappingSurfLeafSize = options_.mappingSurfLeafSize;
    float surroundingKeyframeDensity = options_.surroundingKeyframeDensity;

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    laserPoseGuess = Eigen::Isometry3d::Identity();
    laserPoseCurr = Eigen::Isometry3d::Identity();
    laserPoseLast = Eigen::Isometry3d::Identity();
}

void MapCloudBuilder::performLoopClosure(std::pair<double,double>* loopInfo)
{
    laserLoopDetector_->process(cloudKeyPoses3D, cloudKeyPoses6D, cornerCloudKeyFrames, surfCloudKeyFrames,
            loopIndexQueue, loopPoseQueue, loopNoiseQueue, timeLaserInfoCur, loopInfo);
}

bool MapCloudBuilder::processLaserCloud(const pcl::PointCloud<PointXYZIRT>::Ptr laserCloud, double laserTime, std::vector<PoseSample>& gpsSamples)
{
    std::lock_guard<std::mutex> lock(mtxCloud);

    timeLaserInfoCur = laserTime;
    laserCloudIn = laserCloud;
    ExtractPointCloud();

    if (timeLaserInfoCur - timeLastProcessing >= options_.mappingProcessInterval) {
        timeLastProcessing = timeLaserInfoCur;

        updateInitialGuess();

        extractSurroundingKeyFrames();

        downsampleCurrentScan();

        scan2MapOptimization();

        saveKeyFramesAndFactor(gpsSamples);

        return true;
    }

    return false;
}

void MapCloudBuilder::ExtractPointCloud()
{
    Eigen::Isometry3d *skewPose = nullptr;
    laserCloudExtractor_->process(laserCloudIn, timeLaserInfoCur, skewPose);
    extractedCloud = laserCloudExtractor_->getExtractedCloud();
    laserCloudCornerLast = laserCloudExtractor_->getCornerCloud();
    laserCloudSurfLast = laserCloudExtractor_->getSurfaceCloud();
}


void MapCloudBuilder::updateInitialGuess()
{
    Eigen::Isometry3d poseIncr = laserPoseLast.inverse() * laserPoseCurr;
    laserPoseGuess = laserPoseCurr * poseIncr;
    laserPoseLast = laserPoseCurr;

    // Imu odometry
    // TODO...
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
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)options_.surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
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
        if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > options_.surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
            // transformed cloud available
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        }
        else {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap   += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);

    // clear map cache if too large
    if (laserCloudMapContainer.size() > 1000)
        laserCloudMapContainer.clear();
}

void MapCloudBuilder::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return; 
    extractNearby();
}

void MapCloudBuilder::downsampleCurrentScan()
{
    // Downsample cloud from current scan
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
}

void MapCloudBuilder::scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    laserCloudRegister_->setEdgeFeatureCloud(laserCloudCornerLastDS, laserCloudCornerFromMapDS);
    laserCloudRegister_->setSurfFeatureCloud(laserCloudSurfLastDS, laserCloudSurfFromMapDS);
    laserCloudRegister_->process(laserPoseGuess, laserPoseCurr);
}

bool MapCloudBuilder::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    if (options_.mappingIntervalTime > 0.0) {
        if (timeLaserInfoCur - cloudKeyPoses6D->back().time > options_.mappingIntervalTime)
            return true;
    }

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal(laserPoseCurr.matrix().cast<float>());
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll) < options_.surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < options_.surroundingkeyframeAddingAngleThreshold && 
        abs(yaw) < options_.surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < options_.surroundingkeyframeAddingDistThreshold) {
        return false;
    }

    return true;
}

void MapCloudBuilder::saveKeyFramesAndFactor(std::vector<PoseSample>& gpsSamples)
{
    if (saveFrame() == false)
        return;

    mapPoseOptimizer_->process(cloudKeyPoses3D, cloudKeyPoses6D, timeLaserInfoCur, laserPoseCurr,
        gpsSamples, loopIndexQueue, loopPoseQueue, loopNoiseQueue);

    if (mapPoseOptimizer_->poseUpdated()) {
        // clear map cache
        laserCloudMapContainer.clear();
    }

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

    // save key frame cloud
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    pcl::PointCloud<PointType>::Ptr scCloud;
    if (options_.enableScanContextLoopClosure) {
        // The following code is copy from sc-lio-sam
        // Scan Context loop detector - giseop
        // - SINGLE_SCAN_FULL: using downsampled original point cloud (/full_cloud_projected + downsampling)
        // - SINGLE_SCAN_FEAT: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        // - MULTI_SCAN_FEAT: using NearKeyframes (because a MulRan scan does not have beyond region, so to solve this issue ... )
        const SCInputType sc_input_type = SCInputType::SINGLE_SCAN_FULL; // change this 

        if( sc_input_type == SCInputType::SINGLE_SCAN_FULL ) {
            scCloud = extractedCloud;
        }  
        else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) {
            scCloud = thisSurfKeyFrame;
        }
        //else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) { 
        //    pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(new pcl::PointCloud<PointType>());
        //    loopFindNearKeyframes(multiKeyFrameFeatureCloud, cloudKeyPoses6D->size() - 1, options_.historyKeyframeSearchNum, -1);
        //    scCloud = multiKeyFrameFeatureCloud;
        //}
        if (scCloud.get()) {
            laserLoopDetector_->scManager.makeAndSaveScancontextAndKeys(*scCloud);
        }
    }
}

bool MapCloudBuilder::saveCloudMap(const string& dataDir, float mapResolution)
{
    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;

    string saveMapDirectory;
    if (dataDir.empty())
        saveMapDirectory = std::getenv("HOME") + options_.savePCDDirectory;
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

    float mappingCornerLeafSize = options_.mappingCornerLeafSize;
    float mappingSurfLeafSize = options_.mappingSurfLeafSize;
    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;

    return success;
}

