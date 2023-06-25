
#include "LaserScanAdapt.hpp"


LaserScanAdaptor::LaserScanAdaptor(const SystemParameter& params)
{
    params_ = params;

    allocateMemory();
    resetParameters();
}

void LaserScanAdaptor::allocateMemory()
{
    imuTime = new double[queueLength];
    imuRotX = new double[queueLength];
    imuRotY = new double[queueLength];
    imuRotZ = new double[queueLength];

    laserCloudIn_.reset(new pcl::PointCloud<PointXYZIRT>());
    fullCloud_.reset(new pcl::PointCloud<PointType>());
    extractedCloud_.reset(new pcl::PointCloud<PointType>());

    int N_SCAN = params_.N_SCAN, Horizon_SCAN = params_.Horizon_SCAN;
    fullCloud_->points.resize(N_SCAN*Horizon_SCAN);

    cloudInfo.startRingIndex.assign(N_SCAN, 0);
    cloudInfo.endRingIndex.assign(N_SCAN, 0);
    cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
    cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);
}

void LaserScanAdaptor::resetParameters()
{
    int N_SCAN = params_.N_SCAN, Horizon_SCAN = params_.Horizon_SCAN;

    deskewFlag_ = -1;
    timeScanCur_ = 0;
    timeScanEnd_ = 0;
    laserCloudIn_->clear();
    extractedCloud_->clear();
    rangeMat_ = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    cloudInfo.imuAvailable = false;
    for (int i = 0; i < queueLength; ++i) {
        imuTime[i] = 0;
        imuRotX[i] = 0;
        imuRotY[i] = 0;
        imuRotZ[i] = 0;
    }
    imuPointerCur = 0;

    cloudInfo.odomAvailable = false;
    firstPointFlag = true;
    odomDeskewFlag = false;
}

LaserScanAdaptor::~LaserScanAdaptor()
{
    delete imuTime;
    delete imuRotX;
    delete imuRotY;
    delete imuRotZ;
}

bool LaserScanAdaptor::process(const pcl::PointCloud<PointXYZIRT>::Ptr& laserCloudIn, double laserTime, int deskewFlag,
        const std::vector<ImuSample>& imuSamples, const std::vector<PoseSample>& poseSamples)
{
    resetParameters();

    laserCloudIn_ = laserCloudIn;
    timeScanCur_ = laserTime;
    timeScanEnd_ = timeScanCur_ + laserCloudIn_->points.back().time;
    deskewFlag_ = deskewFlag;

    if (!deskewInfo(imuSamples, poseSamples)) {
        return false;
    }

    projectPointCloud();

    cloudExtraction();

    return true;
}

bool LaserScanAdaptor::deskewInfo(const std::vector<ImuSample>& imuSamples, const std::vector<PoseSample>& poseSamples)
{
    imuDeskewInfo(imuSamples);

    odomDeskewInfo(poseSamples);

    return true;
}

void LaserScanAdaptor::imuDeskewInfo(const std::vector<ImuSample>& imuSamples)
{
    cloudInfo.imuAvailable = false;

    if (imuSamples.empty())
        return;
    imuPointerCur = 0;

    for (int i = 0; i < (int)imuSamples.size(); ++i)
    {
        const ImuSample& imuSample = imuSamples[i];
        double currImuTime = imuSample.timestamp_;

        if (currImuTime < timeScanCur_ - 0.01)
            continue;
        else if (currImuTime > timeScanEnd_ + 0.01)
            break;

        // get roll, pitch, and yaw estimation for this scan
        if (params_.imuType == 0 && currImuTime <= timeScanCur_) {
            cloudInfo.imuRollInit = imuSample.angularRPY_[0];
            cloudInfo.imuPitchInit = imuSample.angularRPY_[1];
            cloudInfo.imuYawInit = imuSample.angularRPY_[2];
        }

        if (imuPointerCur == 0) {
            imuRotX[0] = 0;
            imuRotY[0] = 0;
            imuRotZ[0] = 0;
            imuTime[0] = currImuTime;
            ++imuPointerCur;
            continue;
        }

        // get angular velocity
        double angular_x = imuSample.angularVelocity_.x();
        double angular_y = imuSample.angularVelocity_.y();
        double angular_z = imuSample.angularVelocity_.z();

        // integrate rotation
        double timeDiff = currImuTime - imuTime[imuPointerCur-1];
        imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
        imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
        imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
        imuTime[imuPointerCur] = currImuTime;
        ++imuPointerCur;
    }

    --imuPointerCur;
    if (imuPointerCur <= 0)
        return;

    cloudInfo.imuAvailable = true;
}

void LaserScanAdaptor::odomDeskewInfo(const std::vector<PoseSample>& poseSamples)
{
    cloudInfo.odomAvailable = false;

    if (poseSamples.empty())
        return;

    // get start odometry at the beinning of the scan
    if (poseSamples.front().timestamp_ > timeScanCur_)
        return;

    PoseSample startPose;
    for (int i = 0; i < (int)poseSamples.size(); ++i) {
        startPose = poseSamples[i];
        if (startPose.timestamp_ < timeScanCur_)
            continue;
        else
            break;
    }

    // Initial guess used in mapOptimization
    cloudInfo.initialGuessX = startPose.position_.x();
    cloudInfo.initialGuessY = startPose.position_.y();
    cloudInfo.initialGuessZ = startPose.position_.z();
    cloudInfo.initialGuessRoll = startPose.angularRPY_[0];
    cloudInfo.initialGuessPitch = startPose.angularRPY_[1];
    cloudInfo.initialGuessYaw = startPose.angularRPY_[2];

    cloudInfo.odomAvailable = true;

    // get end odometry at the end of the scan
    odomDeskewFlag = false;

    if (poseSamples.back().timestamp_ < timeScanEnd_)
        return;

    PoseSample endPose;
    for (int i = 0; i < (int)poseSamples.size(); ++i) {
        endPose = poseSamples[i];
        if (endPose.timestamp_ < timeScanEnd_)
            continue;
        else
            break;
    }

    if (int(round(startPose.covariance_[0])) != int(round(endPose.covariance_[0])))
        return;

    Eigen::Affine3f transBegin = pcl::getTransformation(startPose.position_.x(), startPose.position_.y(), startPose.position_.z(),
        startPose.angularRPY_[0], startPose.angularRPY_[1], startPose.angularRPY_[2]);
    Eigen::Affine3f transEnd = pcl::getTransformation(endPose.position_.x(), endPose.position_.y(), endPose.position_.z(),
        endPose.angularRPY_[0], endPose.angularRPY_[1], endPose.angularRPY_[2]);
    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;
    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    odomDeskewFlag = true;
}

void LaserScanAdaptor::findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

    int imuPointerFront = 0;
    while (imuPointerFront < imuPointerCur) {
        if (pointTime < imuTime[imuPointerFront])
            break;
        ++imuPointerFront;
    }

    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
        *rotXCur = imuRotX[imuPointerFront];
        *rotYCur = imuRotY[imuPointerFront];
        *rotZCur = imuRotZ[imuPointerFront];
    }
    else {
        int imuPointerBack = imuPointerFront - 1;
        double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
        *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
        *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
    }
}

void LaserScanAdaptor::findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
{
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

    // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

    // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
    //     return;

    // float ratio = relTime / (timeScanEnd_ - timeScanCur_);

    // *posXCur = ratio * odomIncreX;
    // *posYCur = ratio * odomIncreY;
    // *posZCur = ratio * odomIncreZ;
}

PointType LaserScanAdaptor::deskewPoint(PointType *point, double relTime)
{
    if (deskewFlag_ == -1 || !cloudInfo.imuAvailable)
        return *point;

    double pointTime = timeScanCur_ + relTime;

    float rotXCur, rotYCur, rotZCur;
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

    float posXCur, posYCur, posZCur;
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    if (firstPointFlag) {
        transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
        firstPointFlag = false;
    }

    // transform points to start
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;

    PointType newPoint;
    newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
    newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
    newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
    newPoint.intensity = point->intensity;

    return newPoint;
}

void LaserScanAdaptor::projectPointCloud()
{
    int N_SCAN = params_.N_SCAN, Horizon_SCAN = params_.Horizon_SCAN;
    int downsampleRate = params_.downsampleRate, point_filter_num = params_.point_filter_num;
    float lidarMinRange = params_.lidarMinRange, lidarMaxRange = params_.lidarMaxRange;
    SensorType sensor = params_.sensor;

    vector<int> columnIdnCountVec;
    columnIdnCountVec.assign(N_SCAN, 0);

    // range image projection
    int cloudSize = laserCloudIn_->points.size();
    for (int i = 0; i < cloudSize; ++i) {
        PointType thisPoint;
        thisPoint.x = laserCloudIn_->points[i].x;
        thisPoint.y = laserCloudIn_->points[i].y;
        thisPoint.z = laserCloudIn_->points[i].z;
        thisPoint.intensity = laserCloudIn_->points[i].intensity;

        float range = pointDistance(thisPoint);
        if (range < lidarMinRange || range > lidarMaxRange)
            continue;

        int rowIdn = laserCloudIn_->points[i].ring;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        if (rowIdn % downsampleRate != 0)
            continue;

        if (i % point_filter_num != 0)
            continue;

        int columnIdn = -1;
        if (sensor == SensorType::LIVOX) {
            columnIdn = columnIdnCountVec[rowIdn];
            columnIdnCountVec[rowIdn] += 1;
        }
        else {
            const float ang_res_x = 360.0/float(Horizon_SCAN);
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
        }
        
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        if (rangeMat_.at<float>(rowIdn, columnIdn) != FLT_MAX)
            continue;

        thisPoint = deskewPoint(&thisPoint, laserCloudIn_->points[i].time);

        rangeMat_.at<float>(rowIdn, columnIdn) = range;

        int index = columnIdn + rowIdn * Horizon_SCAN;
        fullCloud_->points[index] = thisPoint;
    }
}

void LaserScanAdaptor::cloudExtraction()
{
    int N_SCAN = params_.N_SCAN, Horizon_SCAN = params_.Horizon_SCAN;
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i) {
        cloudInfo.startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j)  {
            if (rangeMat_.at<float>(i,j) != FLT_MAX) {
                // mark the points' column index for marking occlusion later
                cloudInfo.pointColInd[count] = j;
                // save range info
                cloudInfo.pointRange[count] = rangeMat_.at<float>(i,j);
                // save extracted cloud
                extractedCloud_->push_back(fullCloud_->points[j + i*Horizon_SCAN]);
                // size of extracted cloud
                ++count;
            }
        }
        cloudInfo.endRingIndex[i] = count -1 - 5;
    }
    cloudInfo.extractedCloud = extractedCloud_;
}


