
#include "FeatureCalculate.hpp"


FeatureCalculator::FeatureCalculator(const SystemParameter& params)
{
    params_ = params;

    float odometrySurfLeafSize = params_.odometrySurfLeafSize;
    downSizeFilter_.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    int N_SCAN = params_.N_SCAN, Horizon_SCAN = params_.Horizon_SCAN;
    cloudSmoothness_.resize(N_SCAN*Horizon_SCAN);
    cloudCurvature_ = new float[N_SCAN*Horizon_SCAN];
    cloudNeighborPicked_ = new int[N_SCAN*Horizon_SCAN];
    cloudLabel_ = new int[N_SCAN*Horizon_SCAN];
}

bool FeatureCalculator::process(PointCloudInfo& cloudInfo)
{
    calculateSmoothness(cloudInfo);

    markOccludedPoints(cloudInfo);

    extractFeatures(cloudInfo);

    return true;
}

void FeatureCalculator::calculateSmoothness(PointCloudInfo& cloudInfo)
{
    pcl::PointCloud<PointType>::Ptr extractedCloud = cloudInfo.extractedCloud;
    int cloudSize = extractedCloud->points.size();

    for (int i = 5; i < cloudSize - 5; i++) {
        float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                        + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                        + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                        + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                        + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                        + cloudInfo.pointRange[i+5];            

        cloudCurvature_[i] = diffRange*diffRange;//diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudNeighborPicked_[i] = 0;
        cloudLabel_[i] = 0;
        // cloudSmoothness_ for sorting
        cloudSmoothness_[i].value = cloudCurvature_[i];
        cloudSmoothness_[i].ind = i;
    }
}

void FeatureCalculator::markOccludedPoints(PointCloudInfo& cloudInfo)
{
    pcl::PointCloud<PointType>::Ptr extractedCloud = cloudInfo.extractedCloud;
    int cloudSize = extractedCloud->points.size();

    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i) {
        // occluded points
        float depth1 = cloudInfo.pointRange[i];
        float depth2 = cloudInfo.pointRange[i+1];
        int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

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
        float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
        float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));
        if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
            cloudNeighborPicked_[i] = 1;
    }
}

void FeatureCalculator::extractFeatures(PointCloudInfo& cloudInfo)
{
    pcl::PointCloud<PointType>::Ptr extractedCloud = cloudInfo.extractedCloud; // 当前雷达帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr cornerCloud; // 当前雷达帧提取的角点点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud; // 当前雷达帧提取的平面点点云
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < params_.N_SCAN; i++)
    {
        surfaceCloudScan->clear();

        for (int j = 0; j < 6; j++) {
            int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
            int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloudSmoothness_.begin()+sp, cloudSmoothness_.begin()+ep, ByValue());

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSmoothness_[k].ind;
                if (cloudNeighborPicked_[ind] == 0 && cloudCurvature_[ind] > params_.edgeThreshold) {
                    largestPickedNum++;
                    if (largestPickedNum <= 20) {
                        cloudLabel_[ind] = 1;
                        cornerCloud->push_back(extractedCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked_[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                int ind = cloudSmoothness_[k].ind;
                if (cloudNeighborPicked_[ind] == 0 && cloudCurvature_[ind] < params_.surfThreshold) {
                    cloudLabel_[ind] = -1;
                    cloudNeighborPicked_[ind] = 1;

                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
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

    cloudInfo.cornerCloud = cornerCloud;
    cloudInfo.surfaceCloud = surfaceCloud;
}

