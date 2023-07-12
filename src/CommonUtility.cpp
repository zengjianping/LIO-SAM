
#include "CommonUtility.hpp"


Eigen::Quaterniond eulerAngleToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
Eigen::Vector3d quaternionToEulerAngle(const Eigen::Quaterniond& q)
{
    double roll, pitch, yaw;
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(roll, pitch, yaw);
}

void EntityPose::init()
{
    timestamp = -1;
    index = -1;
    orientation = Eigen::Quaterniond::Identity();
    position = Eigen::Vector3d(0,0,0);
    angular = Eigen::Vector3d(0,0,0);
    angularVel = Eigen::Vector3d(0,0,0);
    linearVel = Eigen::Vector3d(0,0,0);
    linearAcc = Eigen::Vector3d(0,0,0);
    //covariance;
}

void EntityPose::calculateAngular()
{
    //angular = orientation.toRotationMatrix().eulerAngles(0,1,2);
    //angular = orientation.toRotationMatrix().eulerAngles(3,2,1);
    angular = quaternionToEulerAngle(orientation);
}

EntityPose::EntityPose()
{
    init();
}

EntityPose::EntityPose(const Eigen::Quaterniond& _orientation, const Eigen::Vector3d& _position)
{
    init();
    orientation = _orientation;
    position = _position;
    calculateAngular();
}

EntityPose::EntityPose(const Eigen::Isometry3d& transform)
{
    init();
    Eigen::Matrix3d rotmat = transform.rotation();
    orientation = Eigen::Quaterniond(rotmat);
    position = transform.translation();
    //angular = rotmat.eulerAngles(0,1,2);
    calculateAngular();
}

EntityPose::EntityPose(const Eigen::Affine3d& transform)
{
    init();
    Eigen::Matrix3d rotmat = transform.rotation();
    orientation = Eigen::Quaterniond(rotmat);
    position = transform.translation();
    //angular = rotmat.eulerAngles(0,1,2);
    calculateAngular();
}

EntityPose::EntityPose(const gtsam::Pose3& transform)
{
    init();
    orientation = transform.rotation().toQuaternion();
    position = transform.translation();
    //angular = transform.rotation().rpy();
    calculateAngular();
}

void EntityPose::updateFrom(const gtsam::Pose3& transform)
{
    orientation = transform.rotation().toQuaternion();
    position = transform.translation();
    //angular = transform.rotation().rpy();
    calculateAngular();
}

EntityPose::EntityPose(const gtsam::NavState& transform)
{
    init();
    orientation = transform.quaternion();
    position = transform.position();
    //angular = transform.attitude().rpy();
    calculateAngular();
}

Eigen::Isometry3d EntityPose::toIsometry() const
{
    Eigen::Isometry3d transform;
    transform.linear() = orientation.toRotationMatrix();
    transform.translation() = position;
    return transform;
}

Eigen::Affine3d EntityPose::toAffine() const
{
    Eigen::Isometry3d transform;
    transform.linear() = orientation.toRotationMatrix();
    transform.translation() = position;
    return transform;
}

gtsam::Pose3 EntityPose::toGtsamPose() const
{
    return gtsam::Pose3(gtsam::Rot3(orientation), gtsam::Point3(position));
}

gtsam::NavState EntityPose::toGtsamState() const
{
    return gtsam::NavState(gtsam::Rot3(orientation), gtsam::Point3(position), gtsam::Velocity3(0,0,0));
}

EntityPose EntityPose::inverse() const
{
    Eigen::Isometry3d transform = toIsometry().inverse();
    return EntityPose(transform);
}

EntityPose EntityPose::operator *(const EntityPose& other) const
{
    Eigen::Isometry3d transform = toIsometry() * other.toIsometry();
    return EntityPose(transform);
}

EntityPose EntityPose::betweenTo(const EntityPose& other) const
{
    Eigen::Isometry3d transform = toIsometry().inverse() * other.toIsometry();
    return EntityPose(transform);
}

PointType pose3DFromPose6D(const PointTypePose& pose6D)
{
    PointType pose3D;
    pose3D.x = pose6D.x;
    pose3D.y = pose6D.y;
    pose3D.z = pose6D.z;
    pose3D.intensity = pose6D.intensity;
    return pose3D;
}

PointType pose3DFromPose(const EntityPose& pose)
{
    PointType pose3D;
    pose3D.x = pose.position.x();
    pose3D.y = pose.position.y();
    pose3D.z = pose.position.z();
    pose3D.intensity = pose.index;
    return pose3D;
}

PointTypePose pose6DFromPose(const EntityPose& pose)
{
    PointTypePose pose6D;
    pose6D.x = pose.position.x();
    pose6D.y = pose.position.y();
    pose6D.z = pose.position.z();
    pose6D.time = pose.timestamp;
    pose6D.intensity = pose.index;
    pose6D.roll = pose.angular.x();
    pose6D.pitch = pose.angular.y();
    pose6D.yaw = pose.angular.z();
    return pose6D;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, const EntityPose& transform)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Isometry3f transCur = transform.toIsometry().cast<float>();
    
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


