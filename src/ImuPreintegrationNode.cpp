#include "NodeUtility.hpp"
#include "ImuOdometry.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)


class IMUPreintegrationNode : public RosBaseNode
{
public:
    std::mutex mtx;

    //ros::NodeHandle nh; // 节点句柄
    ros::Subscriber subImu; // 订阅IMU原始数据
    ros::Subscriber subOdometry; // 订阅Lidar里程计（来自mapOptimization)
    ros::Publisher pubImuOdometry; // 发布IMU里程计

    // imuQueOpt用来给imuIntegratorOpt_提供数据来源。将当前激光里程计之前的数据统统做积分，优化出bias
    std::deque<sensor_msgs::Imu> imuQueOpt;
    // imuQueImu用来给imuIntegratorImu_提供数据来源。优化当前激光里程计之后，下一帧激光里程计到来之前的位姿
    std::deque<sensor_msgs::Imu> imuQueImu;

    // T_bl: tramsform points from lidar frame to imu frame 
    gtsam::Pose3 imu2Lidar;
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3 lidar2Imu;

    const double delta_t = 0; // 在做IMU数据和雷达里程计同步过程中的时间间隔

    boost::shared_ptr<ImuOdometryIntegrator> imuOdometryIntegrator;
    boost::shared_ptr<ImuOdometryOptimizer> imuOdometryOptimizer;

    IMUPreintegrationNode()
    {
        subImu = nh.subscribe<sensor_msgs::Imu>  (imuTopic, 2000, &IMUPreintegrationNode::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5, &IMUPreintegrationNode::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        
        imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
        lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

        imuOdometryIntegrator.reset(new ImuOdometryIntegrator(*this));
        imuOdometryOptimizer.reset(new ImuOdometryOptimizer(*this));
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty()) {
            return;
        }

        // 1. integrate imu data and optimize
        std::vector<ImuSample> imuSamples;
        while (!imuQueOpt.empty()) {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t) {
                ImuSample imuSample = imuSampleFromSensorMsg(*thisImu);
                imuSamples.push_back(imuSample);
                imuQueOpt.pop_front();
            } else {
                break;
            }
        }

        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsamPoseFromOdmetryMsg(odomMsg);
        gtsam::Pose3 imuPose = lidarPose.compose(lidar2Imu);

        if (!imuOdometryOptimizer->process(imuPose, imuSamples, degenerate)) {
            return;
        }

        // 2. after optiization, re-propagate imu odometry preintegration
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty()) {
            double imuTime = ROS_TIME(&imuQueImu.front());
            if(imuTime < currentCorrectionTime - delta_t) {
                lastImuQT = imuTime;
                imuQueImu.pop_front();
            } else {
                break;
            }
        }
        // integrate imu message from the beginning of this optimization
        imuSamples.clear();
        for (int i = 0; i < (int)imuQueImu.size(); ++i) {
            sensor_msgs::Imu *thisImu = &imuQueImu[i];
            ImuSample imuSample = imuSampleFromSensorMsg(*thisImu);
            imuSamples.push_back(imuSample);
        }

        const gtsam::imuBias::ConstantBias odomBias = imuOdometryOptimizer->getOdometryBias();
        imuOdometryIntegrator->reset(odomBias, imuSamples, lastImuQT);
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConvertOnLidar(*imu_raw, this->extRot, this->extQRPY);
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (!imuOdometryOptimizer->odometryInitialized()) {
            return;
        }

        // predict odometry
        const gtsam::NavState& odomState = imuOdometryOptimizer->getOdomeryState();
        const gtsam::imuBias::ConstantBias& odomBias = imuOdometryOptimizer->getOdometryBias();
        ImuSample imuSample = imuSampleFromSensorMsg(thisImu);
        gtsam::NavState currState = imuOdometryIntegrator->predict(odomState, odomBias, imuSample);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currState.quaternion(), currState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currState.velocity().x();
        odometry.twist.twist.linear.y = currState.velocity().y();
        odometry.twist.twist.linear.z = currState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + odomBias.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + odomBias.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + odomBias.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegrationNode ImuP;

    ROS_INFO("\033[1;32m----> IMU Preintegration Node Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}

