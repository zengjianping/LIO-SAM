
#include "CoreUtility.hpp"



ImuSample::ImuSample()
{
    timestamp_ = 0;
    linearAcceleration_ = Eigen::Vector3d(0,0,0);
    angularRPY_ = Eigen::Vector3d(0,0,0);
    angularVelocity_ = Eigen::Vector3d(0,0,0);
}

PoseSample::PoseSample()
{
    timestamp_ = 0;
    angularRPY_ = Eigen::Vector3d(0,0,0);
    position_ = Eigen::Vector3d(0,0,0);
}

SystemParameter::SystemParameter()
{
}

PointCloudInfo::PointCloudInfo()
{
    timestamp = 0;
    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());
}


