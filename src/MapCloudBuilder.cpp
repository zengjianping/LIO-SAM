
#include "MapCloudBuilder.hpp"


MapCloudBuilder::MapCloudBuilder(const Options& options)
{
    options_  = options;

    float mappingCornerLeafSize = options_.mappingCornerLeafSize;
    float mappingSurfLeafSize = options_.mappingSurfLeafSize;
    float surroundingKeyframeDensity = options_.surroundingKeyframeDensity;

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    mapPoseKeyFrames.reset(new MapPoseFrameVec());
    loopClosureItems.reset(new LoopClosureItemVec());

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
    laserLoopDetector_->process(timeLaserInfoCur, mapPoseKeyFrames, loopClosureItems, loopInfo);
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

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    for (const MapPoseFrame& frame : *mapPoseKeyFrames) {
        cloudKeyPoses3D->points.push_back(pose3DFromPose6D(frame.pose6D));
    }

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
    int numPoses = mapPoseKeyFrames->size();
    for (int i = numPoses-1; i >= 0; --i) {
        const PointTypePose& pose6D = (*mapPoseKeyFrames)[i].pose6D;
        if (timeLaserInfoCur - pose6D.time < 10.0)
            surroundingKeyPosesDS->push_back(pose3DFromPose6D(pose6D));
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
    const PointTypePose& pose6D = mapPoseKeyFrames->back().pose6D;
    PointType pose3D = pose3DFromPose6D(pose6D);
    for (int i = 0; i < (int)cloudToExtract->size(); ++i) {
        if (pointDistance(cloudToExtract->points[i], pose3D) > options_.surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
            // transformed cloud available
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        }
        else {
            // transformed cloud not available
            PointTypePose& pose6D = (*mapPoseKeyFrames)[thisKeyInd].pose6D;
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud((*mapPoseKeyFrames)[thisKeyInd].cornerCloud, &pose6D);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud((*mapPoseKeyFrames)[thisKeyInd].surfCloud, &pose6D);
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
    if (mapPoseKeyFrames->empty() == true)
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
    if (mapPoseKeyFrames->empty())
        return;

    laserCloudRegister_->setEdgeFeatureCloud(laserCloudCornerLastDS, laserCloudCornerFromMapDS);
    laserCloudRegister_->setSurfFeatureCloud(laserCloudSurfLastDS, laserCloudSurfFromMapDS);
    laserCloudRegister_->process(laserPoseGuess, laserPoseCurr);
}

bool MapCloudBuilder::saveFrame()
{
    if (mapPoseKeyFrames->empty())
        return true;

    const PointTypePose& pose6D = mapPoseKeyFrames->back().pose6D;
    if (options_.mappingIntervalTime > 0.0) {
        if (timeLaserInfoCur - pose6D.time > options_.mappingIntervalTime)
            return true;
    }

    Eigen::Affine3f transStart = pclPointToAffine3f(pose6D);
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

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

    // save key frame cloud
    MapPoseFrame frame;
    frame.pose6D = Isometry3dToPclPoint(laserPoseCurr);
    frame.extractedCloud = extractedCloud;
    frame.cornerCloud = thisCornerKeyFrame;
    frame.surfCloud = thisSurfKeyFrame;
    mapPoseKeyFrames->push_back(frame);

    mapPoseOptimizer_->process(timeLaserInfoCur, mapPoseKeyFrames, loopClosureItems, gpsSamples);
    loopClosureItems->clear();

    if (mapPoseOptimizer_->poseUpdated()) {
        // clear map cache
        laserCloudMapContainer.clear();
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

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    for (const MapPoseFrame& frame : *mapPoseKeyFrames) {
        cloudKeyPoses3D->points.push_back(pose3DFromPose6D(frame.pose6D));
        cloudKeyPoses6D->points.push_back(frame.pose6D);
    }
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
        *globalCornerCloud += *transformPointCloud((*mapPoseKeyFrames)[i].cornerCloud, &cloudKeyPoses6D->points[i]);
        *globalSurfCloud   += *transformPointCloud((*mapPoseKeyFrames)[i].surfCloud, &cloudKeyPoses6D->points[i]);
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

