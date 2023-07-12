
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

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}


