#ifndef __OBSTACLE_DETECION__H
#define __OBSTACLE_DETECION__H

#include <vector>
#include <iostream>

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class ObstacleDetection
{
  public:

  // mode enum
  enum MODE {MEAN_DISTANCE, SAMPLEPOINTS};

  ObstacleDetection();
  ObstacleDetection(std::pair<float,float> const);
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
};

std::ostream& operator<<(std::ostream&, ObstacleDetection const&);

#endif //__OBSTACLE_DETECION__H
