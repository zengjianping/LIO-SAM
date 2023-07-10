
#include "MapCloudBuilder.hpp"


MapCloudBuilder::MapCloudBuilder(const Options& options)
{
    options_  = options;

    imuOdometryPredictor_.reset(new ImuOdometryPredictor(options_.optionImuOdomPredictor));
    laserCloudExtractor_.reset(new LaserCloudExtractor(options_.optionCloudExtractor));
    LaserCloudRegister::Type registerType = LaserCloudRegister::NEWTON;
    laserCloudRegister_.reset(LaserCloudRegister::createInstance(registerType, options_.optionCloudRegister));
    laserLoopDetector_.reset(new LaserLoopDetector(options_.optionLoopDetector));
    mapPoseOptimizer_.reset(new MapPoseOptimizer(options_.optionPoseOptimizer));

    float mappingCornerLeafSize = options_.mappingCornerLeafSize;
    float mappingSurfLeafSize = options_.mappingSurfLeafSize;
    float surroundingKeyframeDensity = options_.surroundingKeyframeDensity;

    laserTimeCurr_ = -1;
    laserTimePrev_ = -1;
    laserPoseGuess_ = EntityPose();
    laserPoseCurr_ = EntityPose();
    laserPosePrev_ = EntityPose();

    laserCloudCornerLast_.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudSurfLast_.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    laserCloudCornerLastDS_.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS_.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
    downSizeFilterCorner_.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf_.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    laserCloudCornerFromMap_.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap_.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS_.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS_.reset(new pcl::PointCloud<PointType>());
    downSizeFilterSurroundingKeyPoses_.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    mapPoseKeyFrames_.reset(new MapPoseFrameVec());

    loopClosureItems_.reset(new LoopClosureItemVec());
}

void MapCloudBuilder::processLoopClosure()
{
    std::pair<double,double> loopInfo, *pLoopInfo=nullptr;
    if (consumeLoopInfo(loopInfo)) {
        pLoopInfo = &loopInfo;
    }
    laserLoopDetector_->process(laserTimeCurr_, mapPoseKeyFrames_, loopClosureItems_, pLoopInfo);
}

bool MapCloudBuilder::processLaserCloud(const pcl::PointCloud<PointXYZIRT>::Ptr laserCloud, double laserTime)
{
    if (laserTime - laserTimePrev_ < options_.mappingProcessInterval) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtxCloud_);

    laserTimeCurr_ = laserTime;
    laserCloudIn_ = laserCloud;

    ExtractPointCloud();

    updateInitialGuess();

    extractSurroundingKeyFrames();

    scan2MapOptimization();

    if (saveFrame()) {
        saveKeyFramesAndFactor();
    }

    laserTimePrev_ = laserTimeCurr_;

    return true;
}

void MapCloudBuilder::ExtractPointCloud()
{
    EntityPose *skewPose = nullptr;
    // TODO...

    laserCloudExtractor_->process(laserCloudIn_, laserTimeCurr_, skewPose);
    extractedCloud_ = laserCloudExtractor_->getExtractedCloud();
    laserCloudCornerLast_ = laserCloudExtractor_->getCornerCloud();
    laserCloudSurfLast_ = laserCloudExtractor_->getSurfaceCloud();

    // Downsample cloud from current scan
    laserCloudCornerLastDS_->clear();
    downSizeFilterCorner_.setInputCloud(laserCloudCornerLast_);
    downSizeFilterCorner_.filter(*laserCloudCornerLastDS_);

    laserCloudSurfLastDS_->clear();
    downSizeFilterSurf_.setInputCloud(laserCloudSurfLast_);
    downSizeFilterSurf_.filter(*laserCloudSurfLastDS_);
}

void MapCloudBuilder::updateInitialGuess()
{
    EntityPose poseIncr = laserPosePrev_.betweenTo(laserPoseCurr_);
    laserPoseGuess_ = laserPoseCurr_ * poseIncr;
    laserPosePrev_ = laserPoseCurr_;

    // Imu odometry
    // TODO...
}

void MapCloudBuilder::_extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    for (const MapPoseFrame& frame : *mapPoseKeyFrames_) {
        cloudKeyPoses3D->points.push_back(pose3DFromPose(frame.pose));
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

    downSizeFilterSurroundingKeyPoses_.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses_.filter(*surroundingKeyPosesDS);
    for(auto& pt : surroundingKeyPosesDS->points) {
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }

    // also extract some latest key frames in case the robot rotates in one position
    int numPoses = mapPoseKeyFrames_->size();
    for (int i = numPoses-1; i >= 0; --i) {
        const EntityPose& pose = (*mapPoseKeyFrames_)[i].pose;
        if (laserTimeCurr_ - pose.timestamp < 10.0)
            surroundingKeyPosesDS->push_back(pose3DFromPose(pose));
        else
            break;
    }

    _extractCloud(surroundingKeyPosesDS);
}

void MapCloudBuilder::_extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudCornerFromMap_->clear();
    laserCloudSurfFromMap_->clear();

    const MapPoseFrame& frame = mapPoseKeyFrames_->back();
    PointType pose3D = pose3DFromPose(frame.pose);

    for (int i = 0; i < (int)cloudToExtract->size(); ++i) {
        if (pointDistance(cloudToExtract->points[i], pose3D) > options_.surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer_.find(thisKeyInd) != laserCloudMapContainer_.end()) {
            // transformed cloud available
            *laserCloudCornerFromMap_ += laserCloudMapContainer_[thisKeyInd].first;
            *laserCloudSurfFromMap_  += laserCloudMapContainer_[thisKeyInd].second;
        }
        else {
            // transformed cloud not available
            MapPoseFrame& frame = (*mapPoseKeyFrames_)[thisKeyInd];
            PointTypePose pose6D = pose6DFromPose(frame.pose);
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(frame.cornerCloud, &pose6D);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(frame.surfCloud, &pose6D);
            *laserCloudCornerFromMap_ += laserCloudCornerTemp;
            *laserCloudSurfFromMap_  += laserCloudSurfTemp;
            laserCloudMapContainer_[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner_.setInputCloud(laserCloudCornerFromMap_);
    downSizeFilterCorner_.filter(*laserCloudCornerFromMapDS_);
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf_.setInputCloud(laserCloudSurfFromMap_);
    downSizeFilterSurf_.filter(*laserCloudSurfFromMapDS_);

    // clear map cache if too large
    if (laserCloudMapContainer_.size() > 1000)
        laserCloudMapContainer_.clear();
}

void MapCloudBuilder::extractSurroundingKeyFrames()
{
    if (mapPoseKeyFrames_->empty() == true)
        return;

    _extractNearby();
}

void MapCloudBuilder::scan2MapOptimization()
{
    if (mapPoseKeyFrames_->empty())
        return;

    laserCloudRegister_->setEdgeFeatureCloud(laserCloudCornerLastDS_, laserCloudCornerFromMapDS_);
    laserCloudRegister_->setSurfFeatureCloud(laserCloudSurfLastDS_, laserCloudSurfFromMapDS_);
    laserCloudRegister_->process(laserPoseGuess_, laserPoseCurr_);
}

bool MapCloudBuilder::saveFrame()
{
    if (mapPoseKeyFrames_->empty())
        return true;

    //const PointTypePose& pose6D = mapPoseKeyFrames_->back().pose6D;
    const EntityPose& pose = mapPoseKeyFrames_->back().pose;
    if (options_.mappingIntervalTime > 0.0) {
        //if (laserTimeCurr_ - pose6D.time > options_.mappingIntervalTime)
        if (laserTimeCurr_ - pose.timestamp > options_.mappingIntervalTime)
            return true;
    }

    //Eigen::Affine3f transStart = pclPointToAffine3f(pose6D);
    //Eigen::Affine3f transFinal = laserPoseCurr_.toAffine().cast<float>();
    //Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    //float x, y, z, roll, pitch, yaw;
    //pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    EntityPose transIncr = pose.betweenTo(laserPoseCurr_);
    float roll = transIncr.angular.x(), pitch = transIncr.angular.y(), yaw = transIncr.angular.z();
    float x = transIncr.position.x(), y = transIncr.position.y(), z = transIncr.position.z();

    if (abs(roll) < options_.surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < options_.surroundingkeyframeAddingAngleThreshold && 
        abs(yaw) < options_.surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < options_.surroundingkeyframeAddingDistThreshold) {
        return false;
    }

    return true;
}

void MapCloudBuilder::saveKeyFramesAndFactor()
{
    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS_, *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS_, *thisSurfKeyFrame);

    // save key frame cloud
    MapPoseFrame frame;
    //frame.pose6D = isometry3dToPclPoint(laserPoseCurr_.toIsometry());
    //frame.pose6D.intensity = mapPoseKeyFrames_->size();
    //frame.pose6D.time = laserTimeCurr_;
    frame.pose = laserPoseCurr_;
    frame.pose.index = mapPoseKeyFrames_->size();
    frame.pose.timestamp = laserTimeCurr_;
    frame.extractedCloud = extractedCloud_;
    frame.cornerCloud = thisCornerKeyFrame;
    frame.surfCloud = thisSurfKeyFrame;
    mapPoseKeyFrames_->push_back(frame);

    std::vector<EntityPose> gpsSamples;
    consumeGpsSamples(laserTimeCurr_, gpsSamples);

    mapPoseOptimizer_->process(laserTimeCurr_, mapPoseKeyFrames_, loopClosureItems_, gpsSamples);
    loopClosureItems_->clear();

    if (mapPoseOptimizer_->poseUpdated()) {
        // clear map cache
        laserCloudMapContainer_.clear();
    }

    imuOdometryPredictor_->reset(laserPoseCurr_, false);
}

void MapCloudBuilder::processImuSample(const EntityPose& imuSample)
{
    std::lock_guard<std::mutex> lock(mtxImu_);

    EntityPose imuPose;
    if (imuOdometryPredictor_->predict(imuSample, imuPose)) {
        imuOdomQueue_.push_back(imuPose);
    }
}

void MapCloudBuilder::processGpsSample(const EntityPose& gpsSample)
{
    std::lock_guard<std::mutex> lock(mtxGps_);
    gpsSampleQueue_.push_back(gpsSample);
}

void MapCloudBuilder::consumeGpsSamples(double laserTime, std::vector<EntityPose>& gpsSamples)
{
    std::lock_guard<std::mutex> lock(mtxGps_);

    while (!gpsSampleQueue_.empty()) {
        if (gpsSampleQueue_.front().timestamp < laserTime - 0.2)
            gpsSampleQueue_.pop_front();
        else
            break;
    }

    for (int i = 0; i < (int)gpsSampleQueue_.size(); ++i) {
        const EntityPose& gpsample = gpsSampleQueue_[i];
        if (gpsample.timestamp > laserTime + 0.2)
            break;
        gpsSamples.push_back(gpsample);
    }
}

void MapCloudBuilder::processLoopInfo(const std::pair<double,double>& info)
{
    std::lock_guard<std::mutex> lock(mtxLoop_);

    loopInfoQueue_.push_back(info);
    while (loopInfoQueue_.size() > 5)
        loopInfoQueue_.pop_front();
}

bool MapCloudBuilder::consumeLoopInfo(std::pair<double,double>& info)
{
    std::lock_guard<std::mutex> lock(mtxLoop_);

    if (!loopInfoQueue_.empty()) {
        info = loopInfoQueue_.front();
        return true;
    }
    
    return false;

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
    for (const MapPoseFrame& frame : *mapPoseKeyFrames_) {
        cloudKeyPoses3D->points.push_back(pose3DFromPose(frame.pose));
        cloudKeyPoses6D->points.push_back(pose6DFromPose(frame.pose));
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
    for (int i = 0; i < (int)cloudKeyPoses6D->size(); i++) {
        *globalCornerCloud += *transformPointCloud((*mapPoseKeyFrames_)[i].cornerCloud, &cloudKeyPoses6D->points[i]);
        *globalSurfCloud   += *transformPointCloud((*mapPoseKeyFrames_)[i].surfCloud, &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
    }

    if (mapResolution != 0) {
        cout << "\n\nSave resolution: " << mapResolution << endl;

        // down-sample and save corner cloud
        downSizeFilterCorner_.setInputCloud(globalCornerCloud);
        downSizeFilterCorner_.setLeafSize(mapResolution, mapResolution, mapResolution);
        downSizeFilterCorner_.filter(*globalCornerCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);

        // down-sample and save surf cloud
        downSizeFilterSurf_.setInputCloud(globalSurfCloud);
        downSizeFilterSurf_.setLeafSize(mapResolution, mapResolution, mapResolution);
        downSizeFilterSurf_.filter(*globalSurfCloudDS);
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
    downSizeFilterCorner_.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf_.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;

    return success;
}


