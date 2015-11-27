#ifndef __Subimage_hpp__
#define __Subimage_hpp__

#include "opencv2/core.hpp"
#include "utility.h"

struct Subimage
{
  Subimage(cv::Point const&, cv::Point const&);
  ~Subimage();

  void calculateSubimageValue(cv::Mat const&);

  // member
  cv::Point   tl;
  cv::Point   br;
  cv::Point   roi_center;
  float       value;
};

#endif //__Subimage_hpp__
