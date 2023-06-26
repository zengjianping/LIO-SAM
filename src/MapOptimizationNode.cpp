
#include "nodeUtility.h"
#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"
#include "MapBuilding.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


class MapOptimizationNode : public RosBaseNode
{
public:
    // ROS topic publishers
    ros::Publisher pubLaserCloudSurround; // 发布当前雷达帧周边环境点云
    ros::Publisher pubLaserOdometryGlobal; // 发布雷达里程计（全局）
    ros::Publisher pubLaserOdometryIncremental; // 发布雷达里程计（增量）
    ros::Publisher pubKeyPoses; // 发布3D轨迹
    ros::Publisher pubPath; // 发布6D轨迹
    ros::Publisher pubHistoryKeyFrames; // 发布回环检测的历史关键帧点云
    ros::Publisher pubIcpKeyFrames; // 发布回环检测成功的关键帧点云
    ros::Publisher pubRecentKeyFrames; // 发布局部地图关键帧点云
    ros::Publisher pubRecentKeyFrame; // 发布当前雷达帧的下采样点云
    ros::Publisher pubCloudRegisteredRaw; // 发布当前雷达帧的原始点云
    ros::Publisher pubLoopConstraintEdge; // 发布回环检测结果
    ros::Publisher pubSLAMInfo; // publish SLAM infomation for 3rd-party usage
    ros::Publisher pubGpsOdom;

    // ROS topic suscribers
    ros::Subscriber subCloud; // 订阅从featureExtraction模块发布出来的点云信息集合
    ros::Subscriber subGPS; // 订阅GPS里程计（实际是由robot_localization包计算后的GPS位姿）
    ros::Subscriber subLoop; // 订阅外部回环检测信息

    // ROS services
    ros::ServiceServer srvSaveMap; // 保存地图服务接口

    // 线程锁
    std::mutex mtxGps; // GPS回调函数锁
    std::mutex mtxLoop; // 回环检测线程

    // GPS信息
    std::deque<nav_msgs::Odometry> gpsQueue; // GPS信息队列
    GeographicLib::LocalCartesian gps_trans_;

    PointCloudInfo cloudInfo; // 当前点云信息
    nav_msgs::Path globalPath; // 全局关键帧轨迹
    ros::Time timeLaserInfoStamp; // 当前雷达帧的时间戳
    deque<std_msgs::Float64MultiArray> loopInfoVec; // 外部获取的回环检测配对关系

    // Publish odometry for ROS (incremental)
    bool lastIncreOdomPubFlag = false;
    Eigen::Affine3f increOdomAffine; // incremental odometry in affine

    // publish SLAM infomation for 3rd-party usage
    int lastSLAMInfoPubSize = -1;

    boost::shared_ptr<MapBuilder> mapBuilder_;

public:
    MapOptimizationNode()
    {
        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);
        pubRecentKeyFrames          = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame           = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);
        pubSLAMInfo                 = nh.advertise<lio_sam::cloud_info>("lio_sam/mapping/slam_info", 1);
        pubGpsOdom                  = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/gps_odom", 1);
        pubHistoryKeyFrames         = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames             = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge       = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &MapOptimizationNode::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &MapOptimizationNode::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &MapOptimizationNode::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        srvSaveMap  = nh.advertiseService("lio_sam/save_map", &MapOptimizationNode::saveMapService, this);

        mapBuilder_.reset(new MapBuilder(*this));
    }

    bool saveMapService(lio_sam::save_mapRequest& req, lio_sam::save_mapResponse& res)
    {
        res.success = mapBuilder_->saveCloudMap(req.destination, req.resolution);
        return true;
    }

    void gpsHandler2(const sensor_msgs::NavSatFixConstPtr& gpsMsg)
    {
        if (gpsMsg->status.status != 0)
            return;

        Eigen::Vector3d trans_local_;
        static bool first_gps = false;
        if (!first_gps) {
            first_gps = true;
            gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        }

        gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

        nav_msgs::Odometry gps_odom;
        gps_odom.header = gpsMsg->header;
        gps_odom.header.frame_id = "map";
        gps_odom.pose.pose.position.x = trans_local_[0];
        gps_odom.pose.pose.position.y = trans_local_[1];
        gps_odom.pose.pose.position.z = trans_local_[2];
        gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        pubGpsOdom.publish(gps_odom);
        gpsQueue.push_back(gps_odom);
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        std::lock_guard<std::mutex> lock(mtxGps);
        gpsQueue.push_back(*gpsMsg);
    }

    bool collectGpsSample(double timeScanCur, std::vector<PoseSample>& gpsSamples)
    {
        std::lock_guard<std::mutex> lock(mtxGps);

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeScanCur - 0.2)
                gpsQueue.pop_front();
            else
                break;
        }

        for (int i = 0; i < (int)gpsQueue.size(); ++i)
        {
            nav_msgs::Odometry odomMsg = gpsQueue[i];
            double currOdomTime = ROS_TIME(&odomMsg);
            if (currOdomTime > timeScanCur + 0.2)
                break;
            PoseSample poseSample = poseSampleFromOdometryMsg(odomMsg);
            gpsSamples.push_back(poseSample);
        }

        return true;
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoop);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);
        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    bool popLoopInfo(std::pair<double,double>& info)
    {
        std::lock_guard<std::mutex> lock(mtxLoop);
        if (loopInfoVec.empty())
            return false;

        info.first = loopInfoVec.front().data[0];
        info.second = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();
    }

    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();

            std::pair<double,double> tempInfo;
            bool res = popLoopInfo(tempInfo);
            std::pair<double,double>* loopInfo = res ? &tempInfo : nullptr;

            mapBuilder_->performLoopClosure(loopInfo);
            visualizeLoopClosure();
        }
    }

    void visualizeLoopClosure()
    {
        auto& loopIndexContainer = mapBuilder_->loopIndexContainer;
        auto& copy_cloudKeyPoses6D = mapBuilder_->copy_cloudKeyPoses6D;

        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
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
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
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
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;

        // extract info and feature cloud
        initCloudInfo(msgIn);

        std::vector<PoseSample> gpsSamples;
        collectGpsSample(cloudInfo.timestamp, gpsSamples);

        if (mapBuilder_->processLaserCloud(cloudInfo, gpsSamples))
        {
            publishOdometry();
            publishFrames();
        }
    }

    void initCloudInfo(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfo.timestamp = msgIn->header.stamp.toSec();
        pcl::fromROSMsg(msgIn->cloud_deskewed, *cloudInfo.extractedCloud);
        pcl::fromROSMsg(msgIn->cloud_corner, *cloudInfo.cornerCloud);
        pcl::fromROSMsg(msgIn->cloud_surface, *cloudInfo.surfaceCloud);
        cloudInfo.imuAvailable = msgIn->imuAvailable;
        cloudInfo.odomAvailable = msgIn->odomAvailable;
        cloudInfo.imuRollInit = msgIn->imuRollInit;
        cloudInfo.imuPitchInit = msgIn->imuPitchInit;
        cloudInfo.imuYawInit = msgIn->imuYawInit;
        cloudInfo.initialGuessX = msgIn->initialGuessX;
        cloudInfo.initialGuessY = msgIn->initialGuessY;
        cloudInfo.initialGuessZ = msgIn->initialGuessZ;
        cloudInfo.initialGuessRoll = msgIn->initialGuessRoll;
        cloudInfo.initialGuessPitch = msgIn->initialGuessPitch;
        cloudInfo.initialGuessYaw = msgIn->initialGuessYaw;
    }

    void publishOdometry()
    {
        auto& transformTobeMapped = mapBuilder_->transformTobeMapped;
        auto& incrementalOdometryAffineFront = mapBuilder_->incrementalOdometryAffineFront;
        auto& incrementalOdometryAffineBack = mapBuilder_->incrementalOdometryAffineBack;
        
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        if (lastIncreOdomPubFlag == false) {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        }
        else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true && imuType == 0) {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (mapBuilder_->isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        auto& transformTobeMapped = mapBuilder_->transformTobeMapped;
        auto& cloudKeyPoses3D = mapBuilder_->cloudKeyPoses3D;
        auto& cloudKeyPoses6D = mapBuilder_->cloudKeyPoses6D;
        auto& laserCloudCornerFromMapDS = mapBuilder_->laserCloudCornerFromMapDS;
        auto& laserCloudSurfFromMapDS = mapBuilder_->laserCloudSurfFromMapDS;
        auto& laserCloudCornerLastDS = mapBuilder_->laserCloudCornerLastDS;
        auto& laserCloudSurfLastDS = mapBuilder_->laserCloudSurfLastDS;

        if (cloudKeyPoses3D->points.empty())
            return;

        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);

        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);

        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudInfo.extractedCloud, &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }

        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            // clear path
            globalPath.poses.clear();
            for (int i = 0; i < (int)cloudKeyPoses6D->points.size(); ++i) {
                updatePath(cloudKeyPoses6D->points[i]);
            }
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }

        // publish SLAM infomation for 3rd-party usage
        if (pubSLAMInfo.getNumSubscribers() != 0)
        {
            if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            {
                lio_sam::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                *cloudOut += *laserCloudCornerLastDS;
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudCornerFromMapDS;
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }

        if (savePCD == false)
            return;

        lio_sam::save_mapRequest req;
        lio_sam::save_mapResponse res;

        if(!saveMapService(req, res)){
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        auto& cloudKeyPoses3D = mapBuilder_->cloudKeyPoses3D;
        auto& cloudKeyPoses6D = mapBuilder_->cloudKeyPoses6D;
        auto& cornerCloudKeyFrames = mapBuilder_->cornerCloudKeyFrames;
        auto& surfCloudKeyFrames = mapBuilder_->surfCloudKeyFrames;

        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        //mtxCloud.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        //mtxCloud.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    MapOptimizationNode MO;

    ROS_INFO("\033[1;32m----> Map Optimization Node Started.\033[0m");
    
    std::thread loopthread(&MapOptimizationNode::loopClosureThread, &MO);
    std::thread visualizeMapThread(&MapOptimizationNode::visualizeGlobalMapThread, &MO);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}

