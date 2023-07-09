
#include "CommonUtility.hpp"


void EntityPose::init()
{
    timestamp = -1;
    orientation = Eigen::Quaterniond::Identity();
    position = Eigen::Vector3d(0,0,0);
    angular = Eigen::Vector3d(0,0,0);
    angularVel = Eigen::Vector3d(0,0,0);
    linearVel = Eigen::Vector3d(0,0,0);
    linearAcc = Eigen::Vector3d(0,0,0);
    //covariance;
}

EntityPose::EntityPose()
{
    init();
}

EntityPose::EntityPose(const Eigen::Isometry3d& transform)
{
    init();
    orientation = Eigen::Quaterniond(transform.rotation());
    position = transform.translation();
}

EntityPose::EntityPose(const Eigen::Affine3d& transform)
{
    init();
    orientation = Eigen::Quaterniond(transform.rotation());
    position = transform.translation();
}

EntityPose::EntityPose(const gtsam::Pose3& transform)
{
    init();
    orientation = transform.rotation().toQuaternion();
    position = transform.translation();
}

EntityPose::EntityPose(const gtsam::NavState& transform)
{
    init();
    orientation = transform.quaternion();
    position = transform.position();
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


