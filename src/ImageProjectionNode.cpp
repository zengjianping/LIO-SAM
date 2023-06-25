#include "nodeUtility.h"
#include "lio_sam/cloud_info.h"
#include "LaserScanAdapt.hpp"


class ImageProjectionNode : public RosBaseNode
{
private:
    std::mutex imuLock; // IMU队列线程锁
    std::mutex odoLock; // IMU里程计队列线程锁

    //ros::NodeHandle nh; // 节点句柄
    ros::Subscriber subImu; // 订阅原始IMU数据
    std::deque<sensor_msgs::Imu> imuQueue;
    ros::Subscriber subOdom; // 订阅IMU里程计数据，来自imuPreintegration
    std::deque<nav_msgs::Odometry> odomQueue;
    ros::Subscriber subLaserCloud; // 订阅原始雷达点云数据
    std::deque<sensor_msgs::PointCloud2> cloudQueue;

    ros::Publisher pubExtractedCloud; // 发布去畸变的点云
    // 发布去完畸变后的点云信息
    // 包括：原始点云、去畸变点云、该帧点云的初始旋转旋转角（来自IMU原始roll、pitch、yaw）、该帧点云的初始位姿（来自IMU里程计）
    ros::Publisher pubLaserCloudInfo;
    lio_sam::cloud_info cloudInfoMsg; // 发布的数据结构

    sensor_msgs::PointCloud2 currentCloudMsg; // 从点云队列中提取出当前点云帧做处理
    std_msgs::Header cloudHeader; // 当前雷达帧的header
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;

    int ringFlag = 0;
    int deskewFlag = 0;

    boost::shared_ptr<LaserScanAdaptor> laserScanAdaptor_;

public:
    ImageProjectionNode()
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjectionNode::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjectionNode::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjectionNode::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());

        laserScanAdaptor_.reset(new LaserScanAdaptor(*this));

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConvertOnLidar(*imuMsg, this->extRot, this->extQRPY);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

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

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;
        
        if (!processLaserScan()) {
            return;
        }

        publishClouds();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());

        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
    
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX) {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER) {
            // Convert to Velodyne format
            pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
            tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++) {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else if (sensor == SensorType::MULRAN) {
            // Convert to Velodyne format
            pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
            tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++) {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = static_cast<float>(src.t);
            }
        }
        else if (sensor == SensorType::ROBOSENSE) {
            // Convert to robosense format
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn;
            tmpRobosenseCloudIn.reset(new pcl::PointCloud<RobosensePointXYZIRT>());
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;
            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
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
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // check dense flag
        if (laserCloudIn->is_dense == false) {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        if (ringFlag == 0) {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i) {
                if (currentCloudMsg.fields[i].name == "ring") {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1) {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        if (deskewFlag == 0) {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields) {
                if (field.name == "time" || field.name == "t") {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1) {
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }
        }

        return true;
    }

    bool processLaserScan()
    {
        // get timestamp
        cloudHeader = currentCloudMsg.header;
        double timeScanCur = cloudHeader.stamp.toSec();
        double timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        std::vector<ImuSample> imuSamples;
        if (!collectImuSample(timeScanCur, timeScanEnd, imuSamples)) {
            return false;
        }

        std::vector<PoseSample> poseSamples;
        if (!collectPoseSample(timeScanCur, timeScanEnd, poseSamples)) {
            return false;
        }

        return laserScanAdaptor_->process(laserCloudIn, timeScanCur, deskewFlag, imuSamples, poseSamples);
    }

    bool collectImuSample(double timeScanCur, double timeScanEnd, std::vector<ImuSample>& imuSamples)
    {
        std::lock_guard<std::mutex> lock1(imuLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        bool has9axis = imuType == 0;
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currImuTime = thisImuMsg.header.stamp.toSec();
            if (currImuTime > timeScanEnd + 0.01)
                break;
            ImuSample imuSample = imuSampleFromSensorMsg(thisImuMsg, has9axis);
            imuSamples.push_back(imuSample);;
        }

        return true;
    }

    bool collectPoseSample(double timeScanCur, double timeScanEnd, std::vector<PoseSample>& poseSamples)
    {
        std::lock_guard<std::mutex> lock2(odoLock);

        float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20;
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - sync_diff_time)
                odomQueue.pop_front();
            else
                break;
        }

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            nav_msgs::Odometry odomMsg = odomQueue[i];
            double currOdomTime = ROS_TIME(&odomMsg);
            if (currOdomTime > timeScanCur + sync_diff_time)
                break;
            PoseSample poseSample = poseSampleFromOdometryMsg(odomMsg);
            poseSamples.push_back(poseSample);
        }

        return true;
    }

    void publishClouds()
    {
        const PointCloudInfo& cloudInfo = laserScanAdaptor_->getPointCloudInfo();
        cloudInfoMsg.header = cloudHeader;
        cloudInfoMsg.startRingIndex = cloudInfo.startRingIndex;
        cloudInfoMsg.endRingIndex = cloudInfo.endRingIndex;
        cloudInfoMsg.pointColInd = cloudInfo.pointColInd;
        cloudInfoMsg.pointRange = cloudInfo.pointRange;
        cloudInfoMsg.imuAvailable = cloudInfo.imuAvailable;
        cloudInfoMsg.odomAvailable = cloudInfo.odomAvailable;
        cloudInfoMsg.imuRollInit = cloudInfo.imuRollInit;
        cloudInfoMsg.imuPitchInit = cloudInfo.imuPitchInit;
        cloudInfoMsg.imuYawInit = cloudInfo.imuYawInit;
        cloudInfoMsg.initialGuessX = cloudInfo.initialGuessX;
        cloudInfoMsg.initialGuessY = cloudInfo.initialGuessY;
        cloudInfoMsg.initialGuessZ = cloudInfo.initialGuessZ;
        cloudInfoMsg.initialGuessRoll = cloudInfo.initialGuessRoll;
        cloudInfoMsg.initialGuessPitch = cloudInfo.initialGuessPitch;
        cloudInfoMsg.initialGuessYaw = cloudInfo.initialGuessYaw;
        cloudInfoMsg.cloud_deskewed = publishCloud(pubExtractedCloud, cloudInfo.extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfoMsg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjectionNode IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Node Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}

