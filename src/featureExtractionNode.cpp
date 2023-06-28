#include "NodeUtility.hpp"
#include "lio_sam/cloud_info.h"
#include "FeatureCalculate.hpp"


class FeatureExtractionNode : public RosBaseNode
{
public:
    //ros::NodeHandle nh; // 节点句柄
    ros::Subscriber subLaserCloudInfo; // 订阅去畸变点云信息
    ros::Publisher pubLaserCloudInfo; // 发布增加了角点和平面点的点云信息包
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    lio_sam::cloud_info cloudInfoMsg; // 接收和发布的点云信息包
    std_msgs::Header cloudMsgHeader; // 接收到的当前帧信息的数据头

    boost::shared_ptr<FeatureCalculator> featureCalculator_;


    FeatureExtractionNode()
    {
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtractionNode::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);

        featureCalculator_.reset(new FeatureCalculator(*this));
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfoMsg = *msgIn; // new cloud info
        cloudMsgHeader = msgIn->header; // new cloud header

        PointCloudInfo cloudInfo;
        cloudInfo.timestamp = ROS_TIME(msgIn);
        pcl::fromROSMsg(msgIn->cloud_deskewed, *cloudInfo.extractedCloud); // new cloud for extraction
        cloudInfo.startRingIndex = msgIn->startRingIndex;
        cloudInfo.endRingIndex = msgIn->endRingIndex;
        cloudInfo.pointColInd = msgIn->pointColInd;
        cloudInfo.pointRange = msgIn->pointRange;

        featureCalculator_->process(cloudInfo);

        publishFeatureCloud(cloudInfo);
    }

    void freeCloudInfoMemory()
    {
        cloudInfoMsg.startRingIndex.clear();
        cloudInfoMsg.endRingIndex.clear();
        cloudInfoMsg.pointColInd.clear();
        cloudInfoMsg.pointRange.clear();
    }

    void publishFeatureCloud(const PointCloudInfo& cloudInfo)
    {
        // free cloud info memory
        freeCloudInfoMemory();

        // save newly extracted features
        cloudInfoMsg.cloud_corner  = publishCloud(pubCornerPoints, cloudInfo.cornerCloud, cloudMsgHeader.stamp, lidarFrame);
        cloudInfoMsg.cloud_surface = publishCloud(pubSurfacePoints, cloudInfo.surfaceCloud, cloudMsgHeader.stamp, lidarFrame);

        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfoMsg);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtractionNode FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Node Started.\033[0m");
   
    ros::spin();

    return 0;
}