
#include "nodeUtility.h"


RosBaseNode::RosBaseNode()
{
    nh.param<std::string>("/robot_id", robot_id, "roboat");

    nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
    nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
    nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
    nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

    nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
    nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
    nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
    nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

    nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
    nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
    nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

    nh.param<bool>("lio_sam/savePCD", savePCD, false);
    nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    std::string sensorStr;
    nh.param<std::string>("lio_sam/sensor", sensorStr, "");
    if (sensorStr == "velodyne")
    {
        sensor = SensorType::VELODYNE;
    }
    else if (sensorStr == "ouster")
    {
        sensor = SensorType::OUSTER;
    }
    else if (sensorStr == "livox")
    {
        sensor = SensorType::LIVOX;
    }
    else if  (sensorStr == "robosense")
    {
        sensor = SensorType::ROBOSENSE;
    }
    else if (sensorStr == "mulran")
    {
        sensor = SensorType::MULRAN;
    } 
    else
    {
        ROS_ERROR_STREAM(
            "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
        ros::shutdown();
    }

    nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
    nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
    nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
    nh.param<int>("lio_sam/point_filter_num", point_filter_num, 3);
    nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
    nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

    nh.param<int>("lio_sam/imuType", imuType, 0);
    nh.param<float>("lio_sam/imuRate", imuRate, 500.0);
    nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
    nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
    nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
    nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
    nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
    nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
    nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY).inverse();

    nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
    nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
    nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
    nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

    nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

    nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
    nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

    nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
    nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
    nh.param<float>("liorf/surroundingKeyframeMapLeafSize", surroundingKeyframeMapLeafSize, 0.2);

    nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<float>("lio_sam/loopClosureICPSurfLeafSize", loopClosureICPSurfLeafSize, 0.3);
    nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);
    nh.param<bool>("lio_sam/enableScanContextLoopClosure", enableScanContextLoopClosure, false);

    nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    usleep(100);
}


sensor_msgs::Imu imuConvertOnLidar(const sensor_msgs::Imu& imu_in, const Eigen::Matrix3d& extRot, const Eigen::Quaterniond& extQRPY)
{
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}

