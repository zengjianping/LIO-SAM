#include "NodeUtility.hpp"


class TransformFusionNode : public RosBaseNode
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry; // 订阅IMU里程计
    deque<nav_msgs::Odometry> imuOdomQueue;

    ros::Subscriber subLaserOdometry; // 订阅雷达里程计
    Eigen::Affine3f lidarOdomAffine; // 时时更新的雷达里程计变量
    double lidarOdomTime = -1;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath; // 发布两帧雷达里程计之间的IMU轨迹

    tf::TransformListener tfListener; // tf2的相关组件
    tf::StampedTransform lidar2Baselink;

    // tf broadcaster
    tf::TransformBroadcaster tfMap2Odom;
    tf::TransformBroadcaster tfOdom2BaseLink;

    nav_msgs::Path imuPath;
    double lastPathTime = -1;

    TransformFusionNode()
    {
        if(lidarFrame != baselinkFrame) {
            try {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
        }

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusionNode::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &TransformFusionNode::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath = nh.advertise<nav_msgs::Path>("lio_sam/imu/path", 1);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // publish tf
        //static tf::TransformBroadcaster tfMap2Odom;
        tf::Transform mapToOdom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(mapToOdom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);
        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1) {
            return;
        }
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }

        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        //static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odomToBaselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odomToBaselink);

        // publish IMU path
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - lastPathTime > 0.1)
        {
            lastPathTime = imuTime;
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.stamp = imuOdomQueue.back().header.stamp;
            poseStamped.header.frame_id = odometryFrame;
            poseStamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(poseStamped);
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0) {
                imuPath.poses.erase(imuPath.poses.begin());
            }
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");

    TransformFusionNode TF;

    ROS_INFO("\033[1;32m----> Transform Fusion Node Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}

