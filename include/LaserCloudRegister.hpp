
#ifndef __LASER_FEATURE_REGISTER_H__
#define __LASER_FEATURE_REGISTER_H__

#include "CommonUtility.hpp"


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Declare LaserCloudRegister

class LaserCloudRegister
{
public:
    enum Type {
        NEWTON = 0,
        CERES
    };

    struct Options {
        int edgeFeatureMinValidNum = 10;
        int surfFeatureMinValidNum = 100;
        int maxIterCount = 10;
        int minFeatureNum = 50;
        int ceresDerivative = 0; // 0: automatic, 1: analytic
        bool undistortScan = false;
        float scanPeriod = 0.1;
        bool featureMatchMethod = 0; // 0: fit, 1: search
        float z_tollerance = 1000; 
        float rotation_tollerance = 1000;
    };

    static LaserCloudRegister* createInstance(Type type, const Options& options);

public:
    LaserCloudRegister(const Options& options);
    virtual ~LaserCloudRegister();

public:
    void setEdgeFeatureCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast);
    void setSurfFeatureCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudCurr, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudLast);
    bool process(const EntityPose& poseGuess, EntityPose& poseFinal);

protected:
    virtual bool preprocess(const EntityPose& poseGuess);
    virtual bool postprocess(EntityPose& poseFinal);
    virtual void transformPointToLast(const pcl::PointXYZI& inp, pcl::PointXYZI& outp) = 0;
    virtual bool prepareProcessing() = 0;
    void processEdgeFeatureCloud();
    bool matchOneEdgeFeatureS(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& lastPointA, Eigen::Vector3d& lastPointB);
    virtual void addOneEdgeFeatureS(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lastPointA, const Eigen::Vector3d& lastPointB) = 0;
    bool matchOneEdgeFeatureF(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& lineNorm, Eigen::Vector3d& centPoint);
    virtual void addOneEdgeFeatureF(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lineNorm, const Eigen::Vector3d& centPoint) = 0;
    void processSurfFeatureCloud();
    bool matchOneSurfFeatureS(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& lastPointA, Eigen::Vector3d& lastPointB, Eigen::Vector3d& lastPointC);
    virtual void addOneSurfFeatureS(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& lastPointA, const Eigen::Vector3d& lastPointB, const Eigen::Vector3d& lastPointC) = 0;
    bool matchOneSurfFeatureF(const pcl::PointXYZI& pointOri, pcl::PointXYZI& pointSel, Eigen::Vector3d& currPoint,
        Eigen::Vector3d& planeNorm, double& planeIntercept);
    virtual void addOneSurfFeatureF(const pcl::PointXYZI& pointOri, const pcl::PointXYZI& pointSel, const Eigen::Vector3d& currPoint,
        const Eigen::Vector3d& planeNorm, double planeIntercept) = 0;
    virtual bool solveOptimProblem(int iterCount) = 0;

protected:
    EntityPose getFinalPose() { return poseFinal_; }
    bool getPoseConverged() { return poseConverged_; }
    bool getPoseDegenerated() { return poseDegenerated_; }

protected:
    Options options_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edgeCloudCurr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr edgeCloudLast_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloudCurr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloudLast_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeCloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfCloud_;
    EntityPose poseFinal_;
    bool poseConverged_ = true;
    bool poseDegenerated_ = false;
};

#endif // __LASER_FEATURE_REGISTER_H__


