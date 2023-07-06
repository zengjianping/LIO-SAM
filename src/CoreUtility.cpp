
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


