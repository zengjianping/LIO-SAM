
#ifndef __LASER_CLOUD_EXTRACTOR_H__
#define __LASER_CLOUD_EXTRACTOR_H__

#include "CommonUtility.hpp"


class LaserCloudExtractor
{
public:
    struct Options {
        int N_SCAN;
        int Horizon_SCAN;
        int downsampleRate;
        int pointFilterNum;
        float lidarMinRange;
        float lidarMaxRange;
        bool sequenceColumn = false;
        float surfLeafSize;
        float edgeThreshold;
        float surfThreshold;
    };

public:
    LaserCloudExtractor(const Options& options);
    ~LaserCloudExtractor();

public:
    bool process(const pcl::PointCloud<PointXYZIRT>::Ptr& laserCloud, double laserTime, Eigen::Isometry3d *skewPose);
    pcl::PointCloud<PointType>::Ptr getExtractedCloud() { return extractedCloud_; }
    pcl::PointCloud<PointType>::Ptr getCornerCloud() { return cornerCloud_; }
    pcl::PointCloud<PointType>::Ptr getSurfaceCloud() { return surfaceCloud_; }

protected:
    PointType deskewPoint(const PointType& inPoint, double relTime);
    void prepareProcessing(const pcl::PointCloud<PointXYZIRT>::Ptr& laserCloud, double laserTime, Eigen::Isometry3d *skewPose);
    void projectPointCloud();
    void extractPointCloud();
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();

protected:
    Options options_;

    double timeScanCur_ = 0; // 当前雷达帧的起始时间
    double timeScanEnd_ = 0; // 当前雷达帧的结束时间
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloud_;
    pcl::PointCloud<PointType>::Ptr fullCloud_;
    pcl::PointCloud<PointType>::Ptr extractedCloud_; // 当前雷达帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr cornerCloud_; // 当前雷达帧提取的角点点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud_; // 当前雷达帧提取的平面点点云
    cv::Mat rangeMat_; // 存储点云的range图像
    std::vector<int32_t> startRingIndex;
    std::vector<int32_t> endRingIndex;
    std::vector<int32_t> pointColInd; // point column index in range image
    std::vector<float> pointRange; // point range

    bool deskewPointCloud_ = false;
    Eigen::Quaterniond quaterCurr2Last_;
    Eigen::Vector3d transCurr2Last_;

    struct SMOOTHNESS { 
        float value;
        size_t ind;
    };
    struct ByValue
    { 
        bool operator()(SMOOTHNESS const &left, SMOOTHNESS const &right) { 
            return left.value < right.value;
        }
    };
    pcl::VoxelGrid<PointType> downSizeFilter_; // 用来做平面特征点点云降采样的过滤器
    std::vector<SMOOTHNESS> cloudSmoothness_; // 点云曲率，cloudSmoothness可以用来做排序，所以里面有另一个字段存储index
    float *cloudCurvature_; // 点云曲率，原始数据，顺序不变。长度为N_SCAN*Horizon_SCAN的数组
    int *cloudNeighborPicked_; // 特征提取标志，1表示遮挡、平行，或者已经进行了特征提取的点，0表示未进行特征提取
    int *cloudLabel_; // 1表示角点，-1表示平面点，0表示没有被选择为特征点，同样是一个N_SCAN*Horizon_SCAN长的数组
};

#endif // __LASER_CLOUD_EXTRACTOR_H__

