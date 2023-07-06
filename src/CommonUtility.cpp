
#include "CommonUtility.hpp"


ImuSample::ImuSample() :
    timestamp_(0), angularRPY_(0,0,0),
    linearAcceleration_(0,0,0), angularVelocity_(0,0,0)
{
}

PoseSample::PoseSample() :
    timestamp_(0), angularRPY_(0,0,0), position_(0,0,0)
{
}


