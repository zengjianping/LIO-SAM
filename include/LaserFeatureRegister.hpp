
#ifndef __LASER_FEATURE_REGISTER_H__
#define __LASER_FEATURE_REGISTER_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declare LaserFeatureRegistration

class LaserFeatureRegistration
{
public:
    enum Type {
        NEWTON = 0,
        CERES
    };

    struct Options {
        int maxIterCount = 10;
    };

    static LaserFeatureRegistration* createInstance(Type type, const Options& options);

protected:
    LaserFeatureRegistration(const Options& options);
    virtual ~LaserFeatureRegistration();

public:
    void setEdgeFeatureCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast);
    void setSurfFeatureCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast);
    bool process(const Eigen::Isometry3d& initPose, Eigen::Isometry3d& finalPose);

protected:
    virtual void setInitPose(const Eigen::Isometry3d& initPose) = 0;
    virtual bool getFinalPose(Eigen::Isometry3d& finalPose) = 0;
    virtual void pointAssociateToMap(const pcl::PointXYZI& inp, pcl::PointXYZI& outp) = 0;
    virtual bool prepareProcessing() = 0;
    void processEdgeFeatureCloud();
    bool matchOneEdgeFeature(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& lastPointA, Eigen::Vector3d& lastPointB);
    virtual void addOneEdgeFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lastPointA, const Eigen::Vector3d& lastPointB) = 0;
    void processSurfFeatureCloud();
    bool matchOneSurfFeature(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& planeNorm, double& planeIntercept);
    virtual void addOneSurfFeature(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& planeNorm, double planeIntercept) = 0;
    virtual bool solveOptimProblem(int iterCount, bool& converged) = 0;

protected:
    Options options_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edgeCloudCurr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edgeCloudLast_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloudCurr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloudLast_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeCloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfCloud_;
};

#endif // __LASER_FEATURE_REGISTER_H__


