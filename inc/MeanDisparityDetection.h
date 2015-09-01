#ifndef __MeanDisparityDetection_h__
#define __MeanDisparityDetection_h__

#include <vector>
#include <iostream>

#include "ObstacleDetection.h"

//OPENCV Stuff
#include "opencv2/opencv.hpp"

class MeanDisparityDetection: public ObstacleDetection
{
  public:
  // mode enum
  enum MODE {MEAN_DISTANCE, MEAN_VALUE};

  MeanDisparityDetection(cv::Mat const&);
  ~MeanDisparityDetection();

  // getter
  std::vector<float> getMeanMap() const;
  std::vector<float> getMeanDistanceMap() const;

  /* virtual */ void build(cv::Mat const&, int, int );
  /* virtual */ void detectObstacles();
  /* virtual */ void print_on(std::ostream&) const;

  private:
    std::string                       mTag;
    std::map<int, std::string>        mPositions;
    std::vector<float>                mMeanMap;
    std::vector<float>                mMeanDistanceMap;
    cv::Mat                           mQMat;
};

#endif //__OBSTACLE_DETECION__H
