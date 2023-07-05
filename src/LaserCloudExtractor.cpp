
#include "LaserCloudExtractor.hpp"


LaserCloudExtractor::LaserCloudExtractor(const Options& options)
    : options_(options)
{
    float surfLeafSize = options_.surfLeafSize;
    downSizeFilter_.setLeafSize(surfLeafSize, surfLeafSize, surfLeafSize);

    int N_SCAN = options_.N_SCAN, Horizon_SCAN = options_.Horizon_SCAN;
    cloudSmoothness_.resize(N_SCAN*Horizon_SCAN);
    cloudCurvature_ = new float[N_SCAN*Horizon_SCAN];
    cloudNeighborPicked_ = new int[N_SCAN*Horizon_SCAN];
    cloudLabel_ = new int[N_SCAN*Horizon_SCAN];
}

LaserCloudExtractor::~LaserCloudExtractor()
{
    delete cloudCurvature_;
    delete cloudNeighborPicked_;
    delete cloudLabel_;
}

bool LaserCloudExtractor::process(const pcl::PointCloud<PointXYZIRT>::Ptr& laserCloud, double laserTime, Eigen::Isometry3d *skewPose)
{
    prepareProcessing(laserCloud, laserTime, skewPose);

    projectPointCloud();

    extractPointCloud();

    return true;
}

void LaserCloudExtractor::prepareProcessing(const pcl::PointCloud<PointXYZIRT>::Ptr& laserCloud, double laserTime, Eigen::Isometry3d *skewPose)
{
    laserCloud_.reset(new pcl::PointCloud<PointXYZIRT>());
    fullCloud_.reset(new pcl::PointCloud<PointType>());
    extractedCloud_.reset(new pcl::PointCloud<PointType>());

    int N_SCAN = options_.N_SCAN, Horizon_SCAN = options_.Horizon_SCAN;
    fullCloud_->points.resize(N_SCAN*Horizon_SCAN);
    rangeMat_ = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    startRingIndex.assign(N_SCAN, 0);
    endRingIndex.assign(N_SCAN, 0);
    pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
    pointRange.assign(N_SCAN*Horizon_SCAN, 0);

    laserCloud_ = laserCloud;
    timeScanCur_ = laserTime;
    timeScanEnd_ = timeScanCur_ + laserCloud_->points.back().time;

    deskewPointCloud_ = false;
    if (skewPose) {
        deskewPointCloud_ = true;
        quaterCurr2Last_ = Eigen::Quaterniond(skewPose->rotation());
        transCurr2Last_ = skewPose->translation();
    }
}

PointType LaserCloudExtractor::deskewPoint(const PointType& inp, double relTime)
{
    if (!deskewPointCloud_) {
        return inp;
    }

    double ratio = relTime / (timeScanEnd_ - timeScanCur_);
    Eigen::Quaterniond quaterIdentity = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond quaterPoint2Last = quaterIdentity.slerp(ratio, quaterCurr2Last_);
    Eigen::Vector3d transPoint2Last = transCurr2Last_ * ratio;

	Eigen::Vector3d inPoint(inp.x, inp.y, inp.z);
	Eigen::Vector3d outPoint = quaterPoint2Last * inPoint + transPoint2Last;
    PointType outp;
	outp.x = outPoint.x();
	outp.y = outPoint.y();
	outp.z = outPoint.z();
	outp.intensity = inp.intensity;

    return outp;
}

void LaserCloudExtractor::projectPointCloud()
{
    int N_SCAN = options_.N_SCAN;
    int Horizon_SCAN = options_.Horizon_SCAN;
    int downsampleRate = options_.downsampleRate;
    int pointFilterNum = options_.pointFilterNum;
    float lidarMinRange = options_.lidarMinRange;
    float lidarMaxRange = options_.lidarMaxRange;
    bool sequenceColumn = options_.sequenceColumn;

    std::vector<int> columnIdnCountVec;
    columnIdnCountVec.assign(N_SCAN, 0);

    // range image projection
    int cloudSize = laserCloud_->points.size();
    for (int i = 0; i < cloudSize; ++i) {
        PointType thisPoint;
        thisPoint.x = laserCloud_->points[i].x;
        thisPoint.y = laserCloud_->points[i].y;
        thisPoint.z = laserCloud_->points[i].z;
        thisPoint.intensity = laserCloud_->points[i].intensity;

        float range = pointDistance(thisPoint);
        if (range < lidarMinRange || range > lidarMaxRange)
            continue;

        int rowIdn = laserCloud_->points[i].ring;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        if (rowIdn % downsampleRate != 0)
            continue;

        if (i % pointFilterNum != 0)
            continue;

        int columnIdn = -1;
        if (sequenceColumn) {
            columnIdn = columnIdnCountVec[rowIdn];
            columnIdnCountVec[rowIdn] += 1;
        } else {
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

        thisPoint = deskewPoint(thisPoint, laserCloud_->points[i].time);

        rangeMat_.at<float>(rowIdn, columnIdn) = range;

        int index = columnIdn + rowIdn * Horizon_SCAN;
        fullCloud_->points[index] = thisPoint;
    }
}

void LaserCloudExtractor::extractPointCloud()
{
    int N_SCAN = options_.N_SCAN, Horizon_SCAN = options_.Horizon_SCAN;
    int count = 0;

    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i) {
        startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j)  {
            if (rangeMat_.at<float>(i,j) != FLT_MAX) {
                // mark the points' column index for marking occlusion later
                pointColInd[count] = j;
                // save range info
                pointRange[count] = rangeMat_.at<float>(i,j);
                // save extracted cloud
                extractedCloud_->push_back(fullCloud_->points[j + i*Horizon_SCAN]);
                // size of extracted cloud
                ++count;
            }
        }
        endRingIndex[i] = count -1 - 5;
    }
}


void LaserCloudExtractor::calculateSmoothness()
{
    pcl::PointCloud<PointType>::Ptr extractedCloud = extractedCloud_;
    int cloudSize = extractedCloud->points.size();

    for (int i = 5; i < cloudSize - 5; i++) {
        float diffRange = pointRange[i-5] + pointRange[i-4]
                        + pointRange[i-3] + pointRange[i-2]
                        + pointRange[i-1] - pointRange[i] * 10
                        + pointRange[i+1] + pointRange[i+2]
                        + pointRange[i+3] + pointRange[i+4]
                        + pointRange[i+5];            

        cloudCurvature_[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudNeighborPicked_[i] = 0;
        cloudLabel_[i] = 0;
        // cloudSmoothness_ for sorting
        cloudSmoothness_[i].value = cloudCurvature_[i];
        cloudSmoothness_[i].ind = i;
    }
}

void LaserCloudExtractor::markOccludedPoints()
{
    pcl::PointCloud<PointType>::Ptr extractedCloud = extractedCloud_;
    int cloudSize = extractedCloud->points.size();

    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i) {
        // occluded points
        float depth1 = pointRange[i];
        float depth2 = pointRange[i+1];
        int columnDiff = std::abs(int(pointColInd[i+1] - pointColInd[i]));

        if (columnDiff < 10) {
            // 10 pixel diff in range image
            if (depth1 - depth2 > 0.3) {
                cloudNeighborPicked_[i - 5] = 1;
                cloudNeighborPicked_[i - 4] = 1;
                cloudNeighborPicked_[i - 3] = 1;
                cloudNeighborPicked_[i - 2] = 1;
                cloudNeighborPicked_[i - 1] = 1;
                cloudNeighborPicked_[i] = 1;
            }
            else if (depth2 - depth1 > 0.3) {
                cloudNeighborPicked_[i + 1] = 1;
                cloudNeighborPicked_[i + 2] = 1;
                cloudNeighborPicked_[i + 3] = 1;
                cloudNeighborPicked_[i + 4] = 1;
                cloudNeighborPicked_[i + 5] = 1;
                cloudNeighborPicked_[i + 6] = 1;
            }
        }

        // parallel beam
        float diff1 = std::abs(float(pointRange[i-1] - pointRange[i]));
        float diff2 = std::abs(float(pointRange[i+1] - pointRange[i]));
        if (diff1 > 0.02 * pointRange[i] && diff2 > 0.02 * pointRange[i])
            cloudNeighborPicked_[i] = 1;
    }
}

void LaserCloudExtractor::extractFeatures()
{
    pcl::PointCloud<PointType>::Ptr extractedCloud = extractedCloud_; // 当前雷达帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr cornerCloud; // 当前雷达帧提取的角点点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud; // 当前雷达帧提取的平面点点云
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < options_.N_SCAN; i++)
    {
        surfaceCloudScan->clear();

        for (int j = 0; j < 6; j++) {
            int sp = (startRingIndex[i] * (6 - j) + endRingIndex[i] * j) / 6;
            int ep = (startRingIndex[i] * (5 - j) + endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloudSmoothness_.begin()+sp, cloudSmoothness_.begin()+ep, ByValue());

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSmoothness_[k].ind;
                if (cloudNeighborPicked_[ind] == 0 && cloudCurvature_[ind] > options_.edgeThreshold) {
                    largestPickedNum++;
                    if (largestPickedNum <= 20) {
                        cloudLabel_[ind] = 1;
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked_[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                int ind = cloudSmoothness_[k].ind;
                if (cloudNeighborPicked_[ind] == 0 && cloudCurvature_[ind] < options_.surfThreshold) {
                    cloudLabel_[ind] = -1;
                    cloudNeighborPicked_[ind] = 1;

                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloudLabel_[k] <= 0) {
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }
        }

        surfaceCloudScanDS->clear();
        downSizeFilter_.setInputCloud(surfaceCloudScan);
        downSizeFilter_.filter(*surfaceCloudScanDS);

        *surfaceCloud += *surfaceCloudScanDS;
    }

    cornerCloud_ = cornerCloud;
    surfaceCloud_ = surfaceCloud;
}

