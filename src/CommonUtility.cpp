
#include "CommonUtility.hpp"


ImuItem::ImuItem() :
    timestamp_(0), angularRPY_(0,0,0),
    linearAcceleration_(0,0,0), angularVelocity_(0,0,0)
{
}

PoseItem::PoseItem() :
    timestamp_(0), angularRPY_(0,0,0), position_(0,0,0)
{
}


