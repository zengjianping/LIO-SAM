
#include "CommonNode.hpp"


ConfigParameter::ConfigParameter()
{
}

RosCommonNode::RosCommonNode()
{
    nhandle_.param<std::string>("/robot_id", params_.robot_id, "roboat");

    nhandle_.param<std::string>("lio_sam/pointCloudTopic", params_.pointCloudTopic, "points_raw");
    nhandle_.param<std::string>("lio_sam/imuTopic", params_.imuTopic, "imu_correct");
    nhandle_.param<std::string>("lio_sam/odomTopic", params_.odomTopic, "odometry/imu");
    nhandle_.param<std::string>("lio_sam/gpsTopic", params_.gpsTopic, "odometry/gps");

    nhandle_.param<std::string>("lio_sam/lidarFrame", params_.lidarFrame, "base_link");
    nhandle_.param<std::string>("lio_sam/baselinkFrame", params_.baselinkFrame, "base_link");
    nhandle_.param<std::string>("lio_sam/odometryFrame", params_.odometryFrame, "odom");
    nhandle_.param<std::string>("lio_sam/mapFrame", params_.mapFrame, "map");

    nhandle_.param<bool>("lio_sam/useImuHeadingInitialization", params_.useImuHeadingInitialization, false);
    nhandle_.param<bool>("lio_sam/useGpsElevation", params_.useGpsElevation, false);
    nhandle_.param<float>("lio_sam/gpsCovThreshold", params_.gpsCovThreshold, 2.0);
    nhandle_.param<float>("lio_sam/poseCovThreshold", params_.poseCovThreshold, 25.0);

    nhandle_.param<bool>("lio_sam/savePCD", params_.savePCD, false);
    nhandle_.param<std::string>("lio_sam/savePCDDirectory", params_.savePCDDirectory, "/Downloads/LOAM/");

    std::string sensorStr;
    nhandle_.param<std::string>("lio_sam/sensor", sensorStr, "");
    if (sensorStr == "velodyne") {
        params_.lidarType = LidarType::VELODYNE;
    }
    else if (sensorStr == "ouster") {
        params_.lidarType = LidarType::OUSTER;
    }
    else if (sensorStr == "livox") {
        params_.lidarType = LidarType::LIVOX;
    }
    else if  (sensorStr == "robosense") {
        params_.lidarType = LidarType::ROBOSENSE;
    }
    else if (sensorStr == "mulran") {
        params_.lidarType = LidarType::MULRAN;
    } 
    else {
        ROS_ERROR_STREAM("Invalid lidar type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
        ros::shutdown();
    }

    nhandle_.param<int>("lio_sam/N_SCAN", params_.N_SCAN, 16);
    nhandle_.param<int>("lio_sam/Horizon_SCAN", params_.Horizon_SCAN, 1800);
    nhandle_.param<int>("lio_sam/downsampleRate", params_.downsampleRate, 1);
    nhandle_.param<int>("lio_sam/point_filter_num", params_.pointFilterNum, 3);
    nhandle_.param<float>("lio_sam/lidarMinRange", params_.lidarMinRange, 1.0);
    nhandle_.param<float>("lio_sam/lidarMaxRange", params_.lidarMaxRange, 1000.0);

    nhandle_.param<int>("lio_sam/imuType", params_.imuType, 0);
    nhandle_.param<float>("lio_sam/imuRate", params_.imuRate, 500.0);
    nhandle_.param<float>("lio_sam/imuAccNoise", params_.imuAccNoise, 0.01);
    nhandle_.param<float>("lio_sam/imuGyrNoise", params_.imuGyrNoise, 0.001);
    nhandle_.param<float>("lio_sam/imuAccBiasN", params_.imuAccBiasN, 0.0002);
    nhandle_.param<float>("lio_sam/imuGyrBiasN", params_.imuGyrBiasN, 0.00003);
    nhandle_.param<float>("lio_sam/imuGravity", params_.imuGravity, 9.80511);
    nhandle_.param<float>("lio_sam/imuRPYWeight", params_.imuRPYWeight, 0.01);
    nhandle_.param<vector<double>>("lio_sam/extrinsicRot", params_.extRotV, vector<double>());
    nhandle_.param<vector<double>>("lio_sam/extrinsicRPY", params_.extRPYV, vector<double>());
    nhandle_.param<vector<double>>("lio_sam/extrinsicTrans", params_.extTransV, vector<double>());
    params_.extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(params_.extRotV.data(), 3, 3);
    params_.extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(params_.extRPYV.data(), 3, 3);
    params_.extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(params_.extTransV.data(), 3, 1);
    params_.extQRPY = Eigen::Quaterniond(params_.extRPY).inverse();

    nhandle_.param<float>("lio_sam/edgeThreshold", params_.edgeThreshold, 0.1);
    nhandle_.param<float>("lio_sam/surfThreshold", params_.surfThreshold, 0.1);
    nhandle_.param<int>("lio_sam/edgeFeatureMinValidNum", params_.edgeFeatureMinValidNum, 10);
    nhandle_.param<int>("lio_sam/surfFeatureMinValidNum", params_.surfFeatureMinValidNum, 100);

    nhandle_.param<float>("lio_sam/odometrySurfLeafSize", params_.odometrySurfLeafSize, 0.2);
    nhandle_.param<float>("lio_sam/mappingCornerLeafSize", params_.mappingCornerLeafSize, 0.2);
    nhandle_.param<float>("lio_sam/mappingSurfLeafSize", params_.mappingSurfLeafSize, 0.2);

    nhandle_.param<float>("lio_sam/z_tollerance", params_.z_tollerance, FLT_MAX);
    nhandle_.param<float>("lio_sam/rotation_tollerance", params_.rotation_tollerance, FLT_MAX);

    nhandle_.param<int>("lio_sam/numberOfCores", params_.numberOfCores, 2);
    nhandle_.param<double>("lio_sam/mappingProcessInterval", params_.mappingProcessInterval, 0.15);

    nhandle_.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", params_.surroundingkeyframeAddingDistThreshold, 1.0);
    nhandle_.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", params_.surroundingkeyframeAddingAngleThreshold, 0.2);
    nhandle_.param<float>("lio_sam/surroundingKeyframeDensity", params_.surroundingKeyframeDensity, 1.0);
    nhandle_.param<float>("lio_sam/surroundingKeyframeSearchRadius", params_.surroundingKeyframeSearchRadius, 50.0);
    nhandle_.param<float>("liorf/surroundingKeyframeMapLeafSize", params_.surroundingKeyframeMapLeafSize, 0.2);

    nhandle_.param<bool>("lio_sam/loopClosureEnableFlag", params_.loopClosureEnableFlag, false);
    nhandle_.param<float>("lio_sam/loopClosureFrequency", params_.loopClosureFrequency, 1.0);
    nhandle_.param<float>("lio_sam/loopClosureICPSurfLeafSize", params_.loopClosureICPSurfLeafSize, 0.3);
    nhandle_.param<int>("lio_sam/surroundingKeyframeSize", params_.surroundingKeyframeSize, 50);
    nhandle_.param<float>("lio_sam/historyKeyframeSearchRadius", params_.historyKeyframeSearchRadius, 10.0);
    nhandle_.param<float>("lio_sam/historyKeyframeSearchTimeDiff", params_.historyKeyframeSearchTimeDiff, 30.0);
    nhandle_.param<int>("lio_sam/historyKeyframeSearchNum", params_.historyKeyframeSearchNum, 25);
    nhandle_.param<float>("lio_sam/historyKeyframeFitnessScore", params_.historyKeyframeFitnessScore, 0.3);
    nhandle_.param<bool>("lio_sam/enableScanContextLoopClosure", params_.enableScanContextLoopClosure, false);

    nhandle_.param<float>("lio_sam/globalMapVisualizationSearchRadius", params_.globalMapVisualizationSearchRadius, 1e3);
    nhandle_.param<float>("lio_sam/globalMapVisualizationPoseDensity", params_.globalMapVisualizationPoseDensity, 10.0);
    nhandle_.param<float>("lio_sam/globalMapVisualizationLeafSize", params_.globalMapVisualizationLeafSize, 1.0);

    usleep(100);
}

sensor_msgs::Imu alignImuToLidar(const sensor_msgs::Imu& imuIn, const Eigen::Matrix3d& extRot, const Eigen::Quaterniond& extQRPY)
{
    sensor_msgs::Imu imuOut = imuIn;

    // rotate acceleration
    Eigen::Vector3d acc(imuIn.linear_acceleration.x, imuIn.linear_acceleration.y, imuIn.linear_acceleration.z);
    acc = extRot * acc;
    imuOut.linear_acceleration.x = acc.x();
    imuOut.linear_acceleration.y = acc.y();
    imuOut.linear_acceleration.z = acc.z();

    // rotate gyroscope
    Eigen::Vector3d gyr(imuIn.angular_velocity.x, imuIn.angular_velocity.y, imuIn.angular_velocity.z);
    gyr = extRot * gyr;
    imuOut.angular_velocity.x = gyr.x();
    imuOut.angular_velocity.y = gyr.y();
    imuOut.angular_velocity.z = gyr.z();

    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imuIn.orientation.w, imuIn.orientation.x, imuIn.orientation.y, imuIn.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imuOut.orientation.x = q_final.x();
    imuOut.orientation.y = q_final.y();
    imuOut.orientation.z = q_final.z();
    imuOut.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1) {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imuOut;
}

pcl::PointCloud<PointXYZIRT>::Ptr adaptLaserCloud(sensor_msgs::PointCloud2& laserCloudMsg, LidarType lidarType)
{
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());

    if (lidarType == LidarType::VELODYNE || lidarType == LidarType::LIVOX) {
        pcl::moveFromROSMsg(laserCloudMsg, *laserCloudIn);
    }
    else if (lidarType == LidarType::OUSTER) {
        // Convert to Velodyne format
        pcl::PointCloud<OusterPointXYZIRT>::Ptr ousterCloudIn;
        ousterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        pcl::moveFromROSMsg(laserCloudMsg, *ousterCloudIn);
        laserCloudIn->points.resize(ousterCloudIn->size());
        laserCloudIn->is_dense = ousterCloudIn->is_dense;
        for (size_t i = 0; i < ousterCloudIn->size(); i++) {
            auto &src = ousterCloudIn->points[i];
            auto &dst = laserCloudIn->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.t * 1e-9f;
        }
    }
    else if (lidarType == LidarType::MULRAN) {
        // Convert to Velodyne format
        pcl::PointCloud<MulranPointXYZIRT>::Ptr mulranCloudIn;
        mulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        pcl::moveFromROSMsg(laserCloudMsg, *mulranCloudIn);
        laserCloudIn->points.resize(mulranCloudIn->size());
        laserCloudIn->is_dense = mulranCloudIn->is_dense;
        for (size_t i = 0; i < mulranCloudIn->size(); i++) {
            auto &src = mulranCloudIn->points[i];
            auto &dst = laserCloudIn->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = static_cast<float>(src.t);
        }
    }
    else if (lidarType == LidarType::ROBOSENSE) {
        // Convert to robosense format
        pcl::PointCloud<RobosensePointXYZIRT>::Ptr robosenseCloudIn;
        robosenseCloudIn.reset(new pcl::PointCloud<RobosensePointXYZIRT>());
        pcl::moveFromROSMsg(laserCloudMsg, *robosenseCloudIn);
        laserCloudIn->points.resize(robosenseCloudIn->size());
        laserCloudIn->is_dense = robosenseCloudIn->is_dense;
        double start_stamptime = robosenseCloudIn->points[0].timestamp;
        for (size_t i = 0; i < robosenseCloudIn->size(); i++) {
            auto &src = robosenseCloudIn->points[i];
            auto &dst = laserCloudIn->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.timestamp - start_stamptime;
        }
    } 
    else {
        ROS_ERROR_STREAM("Unknown lidar type: " << int(lidarType));
        ros::shutdown();
    }

    // check dense flag
    if (laserCloudIn->is_dense == false) {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }

    // check ring channel
    int ringFlag = -1;
    for (int i = 0; i < (int)laserCloudMsg.fields.size(); ++i) {
        if (laserCloudMsg.fields[i].name == "ring") {
            ringFlag = 1;
            break;
        }
    }
    if (ringFlag == -1) {
        ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
        ros::shutdown();
    }

    // check point time
    int deskewFlag = -1;
    for (auto &field : laserCloudMsg.fields) {
        if (field.name == "time" || field.name == "t") {
            deskewFlag = 1;
            break;
        }
    }
    if (deskewFlag == -1) {
        ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }

    return laserCloudIn;
}

EntityPose poseFromImuMsg(const sensor_msgs::Imu& imuMsg, bool has9axis)
{
    EntityPose pose;
    double imuTime = ROS_TIME(&imuMsg);
    pose.timestamp = imuTime;
    const auto& linear_acceleration = imuMsg.linear_acceleration;
    pose.linearAcc = Eigen::Vector3d(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);
    const auto& angular_velocity = imuMsg.angular_velocity;
    pose.angularVel = Eigen::Vector3d(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    if (has9axis) {
        double roll, pitch, yaw;
        imuRPY2rosRPY(&imuMsg, &roll, &pitch, &yaw);
        pose.angular = Eigen::Vector3d(roll, pitch, yaw);
    }
    return pose;
}

EntityPose poseFromOdometryMsg(const nav_msgs::Odometry& odomMsg)
{
    EntityPose pose;
    double odomTime = ROS_TIME(&odomMsg);
    pose.timestamp = odomTime;
    pose.position = Eigen::Vector3d(odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.position.z);
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odomMsg.pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    pose.angular = Eigen::Vector3d(roll, pitch, yaw);
    pose.covariance = odomMsg.pose.covariance;
    return pose;
}

