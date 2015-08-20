#ifndef __Samplepoint_hpp__
#define __Samplepoint_hpp__

#include "opencv2/core.hpp"
#include "utility.h"

struct Samplepoint
{
  Samplepoint(cv::Point, int);
  ~Samplepoint();

  void calculateSamplepointValue(cv::Mat const&);

  // member
  cv::Point   center;
  int         radius;
  float       value;
};

#endif //__Samplepoint_hpp__
