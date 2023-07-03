
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


class LaserFeatureRegistration
{
public:
    enum Type {
        NEWTON_OPTIMIZE = 0,
        CERES_AUTOMATIC,
        CERES_ANALYTIC
    };

    struct Options {
        int maxIterCount = 10;
    };

    static LaserFeatureRegistration* createInstance(Type type, const Options& options);

protected:
    LaserFeatureRegistration();
    ~LaserFeatureRegistration();

public:
    void setEdgeFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast);
    void setSurfFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast);

    bool process();

protected:
    void processEdgeFeature();

protected:
    Options options_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edgeCloudCurr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edgeCloudLast_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloudCurr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloudLast_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeCloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfCloud_;
};


void LaserFeatureRegistration::setEdgeFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast)
{
    edgeCloudCurr_ = cloudCurr;
    edgeCloudLast_ = cloudLast;
    kdtreeEdgeCloud_->setInputCloud(edgeCloudLast_);
}

void LaserFeatureRegistration::setSurfFeature(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast)
{
    surfCloudCurr_ = cloudCurr;
    surfCloudLast_ = cloudLast;
    kdtreeSurfCloud_->setInputCloud(surfCloudLast_);
}

bool LaserFeatureRegistration::process()
{
    for (int iterCount=0; iterCount < options_.maxIterCount; iterCount++) {
        processEdgeFeature();
    }
    return true;
}

void LaserFeatureRegistration::processEdgeFeature()
{
}

