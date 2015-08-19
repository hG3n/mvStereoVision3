#ifndef __Samplepoint_hpp__
#define __Samplepoint_hpp__

#include "opencv2/core.hpp"

class Samplepoint
{
public:
  Samplepoint();
  Samplepoint(cv::Point, int);
  ~Samplepoint();

  float getSamplepointValue() const;

private:
  cv::Point   mCenterPoint;
  int         mRadius;
  cv::Mat     mMatrix;
};

#endif //__Samplepoint_hpp__
