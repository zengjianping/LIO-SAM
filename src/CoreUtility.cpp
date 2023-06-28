
#include "CoreUtility.hpp"


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

ImuSample::ImuSample() :
    timestamp_(0), angularRPY_(0,0,0),
    linearAcceleration_(0,0,0), angularVelocity_(0,0,0)
{
}

PoseSample::PoseSample() :
    timestamp_(0), angularRPY_(0,0,0), position_(0,0,0)
{
}


