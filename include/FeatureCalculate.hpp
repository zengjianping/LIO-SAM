
#ifndef _FEATURE_CALCULATE_H_
#define _FEATURE_CALCULATE_H_

#include "CoreUtility.hpp"


struct SMOOTHNESS
{ 
    float value;
    size_t ind;
};

struct ByValue
{ 
    bool operator()(SMOOTHNESS const &left, SMOOTHNESS const &right) { 
        return left.value < right.value;
    }
};

class FeatureCalculator
{
public:
    FeatureCalculator(const SystemParameter& params);

public:
    bool process(PointCloudInfo& cloudInfo);

protected:
    void calculateSmoothness(PointCloudInfo& cloudInfo);
    void markOccludedPoints(PointCloudInfo& cloudInfo);
    void extractFeatures(PointCloudInfo& cloudInfo);

protected:
    SystemParameter params_;
    pcl::VoxelGrid<PointType> downSizeFilter_; // 用来做平面特征点点云降采样的过滤器
    std::vector<SMOOTHNESS> cloudSmoothness_; // 点云曲率，cloudSmoothness可以用来做排序，所以里面有另一个字段存储index
    float *cloudCurvature_; // 点云曲率，原始数据，顺序不变。长度为N_SCAN*Horizon_SCAN的数组
    int *cloudNeighborPicked_; // 特征提取标志，1表示遮挡、平行，或者已经进行了特征提取的点，0表示未进行特征提取
    int *cloudLabel_; // 1表示角点，-1表示平面点，0表示没有被选择为特征点，同样是一个N_SCAN*Horizon_SCAN长的数组
};

#endif //_FEATURE_CALCULATE_H_

