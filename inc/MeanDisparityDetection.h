#ifndef __OBSTACLE_DETECION__H__
#define __OBSTACLE_DETECION__H__

#include <ObstacleDetection.h>

#include <vector>
#include <iostream>

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
