#ifndef __OBSTACLE_DETECION__H__
#define __OBSTACLE_DETECION__H__

#include <vector>
#include <iostream>

//OPENCV Stuff
#include "opencv2/opencv.hpp"

class ObstacleDetection
{
  public:

  // mode enum
  enum MODE {MEAN_DISTANCE, SAMPLEPOINTS};

  ObstacleDetection();
  virtual ~ObstacleDetection();

  // pure virtual functions implemented obstacle detection methods
  virtual void build(cv::Mat const&, int, int) = 0;
  virtual void detectObstacles() = 0;

  virtual void print_on(std::ostream&) const;

  // getter 
  std::pair<float,float> getRange() const;

  // setter
  void setRange(std::pair<float,float> const&);

  protected:
    std::string               mTag;
    std::pair<float,float>    mRange;
    std::pair<float,float>    mRangeDisparity;
};

std::ostream& operator<<(std::ostream&, ObstacleDetection const&);

#endif //__OBSTACLE_DETECION__H
