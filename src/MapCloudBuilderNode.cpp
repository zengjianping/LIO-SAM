
#include "CommonNode.hpp"
#include "MapCloudBuilder.hpp"
#include "lio_sam/save_map.h"
#include "lio_sam/cloud_info.h"


void transferParameters(const ConfigParameter& params, MapCloudBuilder::Options& options)
{
    options.useLoopClosure = params.loopClosureEnableFlag;
    options.usePoseOptimize = params.usePoseOptimize;
    options.useImuData = params.useImuData;
    options.deskewLaserScan = params.deskewLaserScan;
    options.useGpsData = params.useGpsData;
    options.registerType = (LaserCloudRegister::Type)params.registerType;
    options.mappingProcessInterval = params.mappingProcessInterval;
    options.mappingIntervalTime = params.mappingIntervalTime;
    options.mappingCornerLeafSize = params.mappingCornerLeafSize;
    options.mappingSurfLeafSize = params.mappingSurfLeafSize;
    options.surroundingkeyframeAddingDistThreshold = params.surroundingkeyframeAddingDistThreshold; 
    options.surroundingkeyframeAddingAngleThreshold = params.surroundingkeyframeAddingAngleThreshold; 
    options.surroundingKeyframeDensity = params.surroundingKeyframeDensity;
    options.surroundingKeyframeSearchRadius = params.surroundingKeyframeSearchRadius;
    options.savePCD = params.savePCD;
    options.savePCDDirectory = params.savePCDDirectory;

    options.optionCloudExtractor.N_SCAN = params.N_SCAN;
    options.optionCloudExtractor.Horizon_SCAN = params.Horizon_SCAN;
    options.optionCloudExtractor.downsampleRate = params.downsampleRate;
    options.optionCloudExtractor.pointFilterNum = params.pointFilterNum;
    options.optionCloudExtractor.lidarMinRange = params.lidarMinRange;
    options.optionCloudExtractor.lidarMaxRange = params.lidarMaxRange;
    options.optionCloudExtractor.sequenceColumn = false;
    options.optionCloudExtractor.surfLeafSize = params.odometrySurfLeafSize;
    options.optionCloudExtractor.edgeThreshold = params.edgeThreshold;
    options.optionCloudExtractor.surfThreshold = params.surfThreshold;

    options.optionCloudRegister.edgeFeatureMinValidNum = params.edgeFeatureMinValidNum;
    options.optionCloudRegister.surfFeatureMinValidNum = params.surfFeatureMinValidNum;
    options.optionCloudRegister.maxIterCount = params.maxIterCount;
    options.optionCloudRegister.minFeatureNum = params.minFeatureNum;
    options.optionCloudRegister.ceresDerivative = params.ceresDerivative;
    options.optionCloudRegister.undistortScan = params.undistortScan;
    options.optionCloudRegister.scanPeriod = params.scanPeriod;
    options.optionCloudRegister.featureMatchMethod = params.featureMatchMethod;
    options.optionCloudRegister.z_tollerance = params.z_tollerance;
    options.optionCloudRegister.rotation_tollerance = params.rotation_tollerance;

    options.optionPoseOptimizer.useGpsElevation = params.useGpsElevation;
    options.optionPoseOptimizer.gpsCovThreshold = params.gpsCovThreshold;
    options.optionPoseOptimizer.poseCovThreshold = params.poseCovThreshold;

    options.optionLoopDetector.loopClosureICPSurfLeafSize = params.loopClosureICPSurfLeafSize;
    options.optionLoopDetector.historyKeyframeSearchRadius = params.historyKeyframeSearchRadius;
    options.optionLoopDetector.historyKeyframeSearchTimeDiff = params.historyKeyframeSearchTimeDiff;
    options.optionLoopDetector.historyKeyframeSearchNum = params.historyKeyframeSearchNum;
    options.optionLoopDetector.historyKeyframeFitnessScore = params.historyKeyframeFitnessScore;
    options.optionLoopDetector.enableScanContextLoopClosure = params.enableScanContextLoopClosure;

    options.optionImuOdomPredictor.imuRate = params.imuRate;
    options.optionImuOdomPredictor.imuAccNoise = params.imuAccNoise;
    options.optionImuOdomPredictor.imuGyrNoise = params.imuGyrNoise;
    options.optionImuOdomPredictor.imuAccBiasN = params.imuAccBiasN;
    options.optionImuOdomPredictor.imuGyrBiasN = params.imuGyrBiasN;
    options.optionImuOdomPredictor.imuGravity = params.imuGravity;
}

class MapCloudBuilderNode : public RosCommonNode
{
public:
    // ROS topic publishers
    ros::Publisher pubLaserCloudSurround; // 发布当前雷达帧周边环境点云
    ros::Publisher pubLaserOdometryGlobal; // 发布雷达里程计（全局）
    ros::Publisher pubKeyPoses; // 发布3D轨迹
    ros::Publisher pubLaserPath; // 发布6D轨迹
    ros::Publisher pubRecentFrameCloud; // 发布局部地图关键帧点云
    ros::Publisher pubCurrFrameCloudFlt; // 发布当前雷达帧的下采样点云
    ros::Publisher pubCurrFrameCloudRaw; // 发布当前雷达帧的原始点云
    ros::Publisher pubSLAMInfo; // publish SLAM infomation for 3rd-party usage
    ros::Publisher pubHistoryKeyFrames; // 发布回环检测的历史关键帧点云
    ros::Publisher pubIcpKeyFrames; // 发布回环检测成功的关键帧点云
    ros::Publisher pubLoopConstraintEdge; // 发布回环检测结果
    ros::Publisher pubGpsOdometry;

    // ROS topic suscribers
    ros::Subscriber subCloud; // 订阅从featureExtraction模块发布出来的点云信息集合
    ros::Subscriber subLoop; // 订阅外部回环检测信息
    ros::Subscriber subGps; // 订阅GPS里程计（实际是由robot_localization包计算后的GPS位姿）
    ros::Subscriber subImu; // 订阅原始IMU数据

    int numImuSamples = 0;
    EntityPose imuOdometry;

    // ROS services
    ros::ServiceServer srvSaveMap; // 保存地图服务接口

    nav_msgs::Path globalPath; // 全局关键帧轨迹
    ros::Time laserTimeStamp; // 当前雷达帧的时间戳
    std::deque<sensor_msgs::PointCloud2> cloudQueue;

    // publish SLAM infomation for 3rd-party usage
    int lastSLAMInfoPubSize = -1;

    boost::shared_ptr<MapCloudBuilder> mapCloudBuilder_;

    // tf broadcaster
    tf::TransformBroadcaster tfMap2Odom;
    tf::TransformBroadcaster tfOdom2Baselink;
    tf::TransformBroadcaster tfLidar2Baselink;
    tf::TransformBroadcaster tfIOdomToImu;

public:
    MapCloudBuilderNode()
    {
        pubLaserCloudSurround       = nhandle_.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nhandle_.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
        pubKeyPoses                 = nhandle_.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserPath                = nhandle_.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);
        pubRecentFrameCloud         = nhandle_.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubCurrFrameCloudFlt        = nhandle_.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCurrFrameCloudRaw        = nhandle_.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);
        pubSLAMInfo                 = nhandle_.advertise<lio_sam::cloud_info>("lio_sam/mapping/slam_info", 1);
        pubHistoryKeyFrames         = nhandle_.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames             = nhandle_.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge       = nhandle_.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);
        pubGpsOdometry              = nhandle_.advertise<nav_msgs::Odometry> ("lio_sam/mapping/gps_odom", 1);

        subCloud = nhandle_.subscribe<sensor_msgs::PointCloud2>(params_.pointCloudTopic, 5, &MapCloudBuilderNode::laserCloudHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop  = nhandle_.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &MapCloudBuilderNode::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGps   = nhandle_.subscribe<nav_msgs::Odometry> (params_.gpsTopic, 200, &MapCloudBuilderNode::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subImu   = nhandle_.subscribe<sensor_msgs::Imu>(params_.imuTopic, 2000, &MapCloudBuilderNode::imuHandler, this, ros::TransportHints().tcpNoDelay());

        srvSaveMap = nhandle_.advertiseService("lio_sam/save_map", &MapCloudBuilderNode::saveMapService, this);

        MapCloudBuilder::Options options;
        transferParameters(params_, options);
        mapCloudBuilder_.reset(new MapCloudBuilder(options));
    }

    bool saveMapService(lio_sam::save_mapRequest& req, lio_sam::save_mapResponse& res)
    {
        res.success = mapCloudBuilder_->saveCloudMap(req.destination, req.resolution);
        return true;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = alignImuToLidar(*imuMsg, params_.extRot, params_.extQRPY);
        EntityPose imuSample = poseFromImuMsg(thisImu, true);
        imuSample.index = numImuSamples++;

        EntityPose imuState;
        if (mapCloudBuilder_->processImuSample(imuSample, imuState)) {
            imuOdometry = imuState;
        }

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        EntityPose gpsSample = poseFromOdometryMsg(*gpsMsg);
        mapCloudBuilder_->processGpsSample(gpsSample);
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        if (loopMsg->data.size() != 2)
            return;

        std::pair<double,double> info;
        info.first = loopMsg->data[0];
        info.second = loopMsg->data[1];
        mapCloudBuilder_->processLoopInfo(info);
    }

    void loopClosureThread()
    {
        if (!params_.loopClosureEnableFlag)
            return;
        
        ROS_INFO("Loop closure thread started!");

        ros::Rate rate(params_.loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            mapCloudBuilder_->processLoopClosure();
            visualizeLoopClosure();
        }

        ROS_INFO("Loop closure thread ended!");
    }

    void visualizeLoopClosure()
    {
        auto loopIndexContainer = mapCloudBuilder_->getLoopIndexContainer();
        auto mapPoseKeyFrames = mapCloudBuilder_->getMapPoseKeyFrames();

        if (loopIndexContainer.empty())
            return;
        
        std::vector<EntityPose> keyPoses;
        for (const MapPoseFrame& frame : *mapPoseKeyFrames) {
            keyPoses.push_back(frame.pose);
        }

        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = params_.odometryFrame;
        markerNode.header.stamp = laserTimeStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = params_.odometryFrame;
        markerEdge.header.stamp = laserTimeStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            const Eigen::Vector3d& pos1 = keyPoses[key_cur].position;
            p.x = pos1.x(); p.y = pos1.y(); p.z = pos1.z();
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            const Eigen::Vector3d& pos2 = keyPoses[key_pre].position;
            p.x = pos2.x(); p.y = pos2.y(); p.z = pos2.z();
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 1)
            return;

        // convert cloud
        sensor_msgs::PointCloud2 currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        // extract time stamp
        laserTimeStamp = currentCloudMsg.header.stamp;

        pcl::PointCloud<PointXYZIRT>::Ptr laserCloud = adaptLaserCloud(currentCloudMsg, params_.lidarType);

        if (mapCloudBuilder_->processLaserCloud(laserCloud, laserTimeStamp.toSec())) {
            publishOdometry();
            publishFrames();
        }
    }

    void pubTransform()
    {
    }

    void publishOdometry()
    {
        EntityPose laserPoseCurr = mapCloudBuilder_->getLaserPoseCurr();

        double px = laserPoseCurr.position.x();
        double py = laserPoseCurr.position.y();
        double pz = laserPoseCurr.position.z();
        double qx = laserPoseCurr.orientation.x();
        double qy = laserPoseCurr.orientation.y();
        double qz = laserPoseCurr.orientation.z();
        double qw = laserPoseCurr.orientation.w();
        
        // Publish TF
        tf::Transform mapToOdom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(mapToOdom, laserTimeStamp, params_.mapFrame, params_.odometryFrame));

        tf::Transform odomToBase = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(px, py, pz));
        tf::StampedTransform odomToBaselink = tf::StampedTransform(odomToBase, laserTimeStamp, params_.odometryFrame, params_.baselinkFrame);
        tfOdom2Baselink.sendTransform(odomToBaselink);

        if (params_.lidarFrame != params_.baselinkFrame) {
            tf::Transform lidarToBaselink = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
            tfLidar2Baselink.sendTransform(tf::StampedTransform(lidarToBaselink, laserTimeStamp, params_.lidarFrame, params_.baselinkFrame));
        }

        if (params_.useImuData) {
            EntityPose poseIncr = imuOdometry.betweenTo(laserPoseCurr);
            //cout << "TF imu: " << poseIncr.print() << endl;
            double px = poseIncr.position.x();
            double py = poseIncr.position.y();
            double pz = poseIncr.position.z();
            double qx = poseIncr.orientation.x();
            double qy = poseIncr.orientation.y();
            double qz = poseIncr.orientation.z();
            double qw = poseIncr.orientation.w();
            // Publish TF
            tf::Transform transform = tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(px, py, pz));
            tf::StampedTransform odomToImu = tf::StampedTransform(transform, laserTimeStamp, params_.odometryFrame, "imu");
            tfIOdomToImu.sendTransform(odomToImu);
        }

        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = laserTimeStamp;
        laserOdometryROS.header.frame_id = params_.odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = px;
        laserOdometryROS.pose.pose.position.y = py;
        laserOdometryROS.pose.pose.position.z = pz;
        laserOdometryROS.pose.pose.orientation.w = qw;
        laserOdometryROS.pose.pose.orientation.x = qx;
        laserOdometryROS.pose.pose.orientation.y = qy;
        laserOdometryROS.pose.pose.orientation.z = qz;
        pubLaserOdometryGlobal.publish(laserOdometryROS);
    }

    void publishFrames()
    {
        auto laserPoseCurr = mapCloudBuilder_->getLaserPoseCurr();
        auto mapPoseKeyFrames = mapCloudBuilder_->getMapPoseKeyFrames();
        auto extractedCloud = mapCloudBuilder_->getExtractedCloud();
        auto laserCloudCornerFromMapDS = mapCloudBuilder_->getLaserCloudCornerFromMapDS();
        auto laserCloudSurfFromMapDS = mapCloudBuilder_->getLaserCloudSurfFromMapDS();
        auto laserCloudCornerLastDS = mapCloudBuilder_->getLaserCloudCornerLastDS();
        auto laserCloudSurfLastDS = mapCloudBuilder_->getLaserCloudSurfLastDS();

        if (mapPoseKeyFrames->empty())
            return;

        pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        for (const MapPoseFrame& frame : *mapPoseKeyFrames) {
            cloudKeyPoses3D->points.push_back(pose3DFromPose(frame.pose));
            cloudKeyPoses6D->points.push_back(pose6DFromPose(frame.pose));
        }

        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, laserTimeStamp, params_.odometryFrame);

        // Publish surrounding key frames
        publishCloud(pubRecentFrameCloud, laserCloudSurfFromMapDS, laserTimeStamp, params_.odometryFrame);

        // publish registered key frame
        if (pubCurrFrameCloudFlt.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS, laserPoseCurr);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, laserPoseCurr);
            publishCloud(pubCurrFrameCloudFlt, cloudOut, laserTimeStamp, params_.odometryFrame);
        }

        // publish registered high-res raw cloud
        if (pubCurrFrameCloudRaw.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            *cloudOut = *transformPointCloud(extractedCloud, laserPoseCurr);
            publishCloud(pubCurrFrameCloudRaw, cloudOut, laserTimeStamp, params_.odometryFrame);
        }

        // publish laser path
        if (pubLaserPath.getNumSubscribers() != 0) {
            // clear path
            globalPath.poses.clear();
            for (int i = 0; i < (int)mapPoseKeyFrames->size(); ++i) {
                updatePath((*mapPoseKeyFrames)[i].pose);
            }
            globalPath.header.stamp = laserTimeStamp;
            globalPath.header.frame_id = params_.odometryFrame;
            pubLaserPath.publish(globalPath);
        }

        // publish SLAM infomation for 3rd-party usage
        if (pubSLAMInfo.getNumSubscribers() != 0) {
            if (lastSLAMInfoPubSize != cloudKeyPoses6D->size()) {
                lio_sam::cloud_info slamInfo;
                slamInfo.header.stamp = laserTimeStamp;
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                *cloudOut += *laserCloudCornerLastDS;
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, laserTimeStamp, params_.lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, laserTimeStamp, params_.odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudCornerFromMapDS;
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, laserTimeStamp, params_.odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }

    void updatePath(const EntityPose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.timestamp);
        pose_stamped.header.frame_id = params_.odometryFrame;
        pose_stamped.pose.position.x = pose_in.position.x();
        pose_stamped.pose.position.y = pose_in.position.y();
        pose_stamped.pose.position.z = pose_in.position.z();
        //tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        const Eigen::Quaterniond& q = pose_in.orientation;
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void visualizeGlobalMapThread()
    {
        ROS_INFO("Visualize thread started!");

        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }

        ROS_INFO("Visualize thread ended!");

        if (params_.savePCD == false)
            return;

        lio_sam::save_mapRequest req;
        lio_sam::save_mapResponse res;

        if(!saveMapService(req, res)){
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        auto mapPoseKeyFrames = mapCloudBuilder_->getMapPoseKeyFrames();

        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (mapPoseKeyFrames->empty())
            return;

        pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        for (const MapPoseFrame& frame : *mapPoseKeyFrames) {
            cloudKeyPoses3D->points.push_back(pose3DFromPose(frame.pose));
        }

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), params_.globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(params_.globalMapVisualizationPoseDensity, params_.globalMapVisualizationPoseDensity, params_.globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points) {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > params_.globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            MapPoseFrame& frame = (*mapPoseKeyFrames)[thisKeyInd];
            *globalMapKeyFrames += *transformPointCloud(frame.cornerCloud, frame.pose);
            *globalMapKeyFrames += *transformPointCloud(frame.surfCloud, frame.pose);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(params_.globalMapVisualizationLeafSize, params_.globalMapVisualizationLeafSize, params_.globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, laserTimeStamp, params_.odometryFrame);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    MapCloudBuilderNode MO;

    ROS_INFO("\033[1;32m----> Map Cloud Builder Node Started.\033[0m");
    
    std::thread loopthread(&MapCloudBuilderNode::loopClosureThread, &MO);
    std::thread visualizeMapThread(&MapCloudBuilderNode::visualizeGlobalMapThread, &MO);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}

