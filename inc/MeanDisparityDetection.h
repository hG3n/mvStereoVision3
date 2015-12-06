#ifndef __MeanDisparityDetection_h__
#define __MeanDisparityDetection_h__

#include <vector>
#include <iostream>

#include "ObstacleDetection.h"
#include "Subimage.h"

//OPENCV Stuff
#include "opencv2/opencv.hpp"

class MeanDisparityDetection: public ObstacleDetection
{
  public:
    // mode enum
    enum MODE {MEAN_DISTANCE, MEAN_VALUE};

    MeanDisparityDetection();
    ~MeanDisparityDetection();

    void init(cv::Mat const&, cv::Mat const&, float, float);

    // getter
    std::vector<Subimage> getSubimageVec() const;
    std::vector<float> getMeanMap() const;
    std::vector<float> getMeanDistanceMap() const;
    std::pair<int,int> getRange() const;

    // setter
    void setRange(float, float);

    /* virtual */ void build(cv::Mat const&, int, int);
    /* virtual */ void detectObstacles();
    /* virtual */ void print_on(std::ostream&) const;

  private:
    std::string                       mTag;
    std::map<int, std::string>        mPositions;
    std::vector<Subimage>             mSubimageVec;
    std::vector<float>                mMeanMap;
    std::vector<float>                mMeanDistanceMap;
    cv::Mat                           mQ_32F;
    cv::Mat_<cv::Vec3f>               mPointcloud;
    int                               mDetectionMode;
};

#endif //__OBSTACLE_DETECION__H
