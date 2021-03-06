#ifndef __MeanDisparityDetection_h__
#define __MeanDisparityDetection_h__

#include <vector>
#include <iostream>

#include "ObstacleDetection.h"
#include "Subimage.h"
#include "ply.h"

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
    std::vector<Subimage>   getSubimageVec() const;
    std::vector<float>      getMeanMap() const;
    std::vector<float>      getMeanDistanceMap() const;
    std::pair<int,int>      getRange() const;
    std::vector<Subimage>   getFoundObstacles() const;
    int                     getObstacleCounter() const;

    // setter
    void setRange(float, float);

    /* virtual */ void build(cv::Mat const&, int, int);
    /* virtual */ void detectObstacles();
    /* virtual */ void print_on(std::ostream&) const;

  private:
    std::string                       mTag;
    cv::Mat                           mDMap;
    std::map<int, std::string>        mPositions;
    std::vector<Subimage>             mSubimageVec;
    std::vector<float>                mMeanMap;
    std::vector<float>                mMeanDistanceMap;
    cv::Mat                           mQ_32F;
    int                               mDetectionMode;
    std::vector<Subimage>             mFoundObstacles;
    int                               mObstacleCounter;
    std::vector<cv::Mat>              mFoundPoints;
};

#endif //__OBSTACLE_DETECION__H
