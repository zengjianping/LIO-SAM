
#ifndef __LASERSCAN_ADAPT_H__
#define __LASERSCAN_ADAPT_H__

#include "CoreUtility.hpp"


struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT, 
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (uint16_t, ring, ring)(double, timestamp, timestamp)
)

// mulran datasets
struct MulranPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (MulranPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (int, ring, ring)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;


class LaserScanAdaptor
{
public:
    LaserScanAdaptor(const SystemParameter& params);
    ~LaserScanAdaptor();
    void allocateMemory();
    void resetParameters();

public:
    bool process(const pcl::PointCloud<PointXYZIRT>::Ptr& laserCloudIn, double laserTime, int deskewFlag,
            const std::vector<ImuSample>& imuSamples, const std::vector<PoseSample>& poseSamples);
    const PointCloudInfo& getPointCloudInfo() { return cloudInfo; }

protected:
    bool deskewInfo(const std::vector<ImuSample>& imuSamples, const std::vector<PoseSample>& poseSamples);
    void imuDeskewInfo(const std::vector<ImuSample>& imuSamples);
    void odomDeskewInfo(const std::vector<PoseSample>& poseSamples);
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);
    PointType deskewPoint(PointType *point, double relTime);
    void projectPointCloud();
    void cloudExtraction();

protected:
    SystemParameter params_; // 算法参数
    PointCloudInfo cloudInfo; // 点云信息

    // 点云数据
    int deskewFlag_; // 当点云的time/t字段不可用，也就是点云中不包含每个点的时间戳，无法进行去畸变，直接返回原始点云
    double timeScanCur_; // 当前雷达帧的起始时间
    double timeScanEnd_; // 当前雷达帧的结束时间
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn_;
    pcl::PointCloud<PointType>::Ptr fullCloud_;
    pcl::PointCloud<PointType>::Ptr extractedCloud_;
    cv::Mat rangeMat_; // 存储点云的range图像

    // 记录每一帧点云从起始到结束过程所有的IMU数据，imuRotX,Y,Z是对这一段时间内的角速度累加的结果
    const int queueLength = 2000;
    double *imuTime;
    double *imuRotX;
    double *imuRotY;
    double *imuRotZ;
    int imuPointerCur; // 记录每一帧点云起止过程中imuTime、imuRotXYZ的实际数据长度

    bool firstPointFlag; // 处理第一个点时，将该点的旋转取逆，记录到transStartInverse中，后续方便计算旋转的增量
    Eigen::Affine3f transStartInverse;
    bool odomDeskewFlag; // 是否有合适的IMU里程计数据
    float odomIncreX; // 记录从IMU里程计出来的平移增量，用来做平移去畸变，实际中没有使用到
    float odomIncreY;
    float odomIncreZ;
};

#endif // __LASERSCAN_ADAPT_H__

