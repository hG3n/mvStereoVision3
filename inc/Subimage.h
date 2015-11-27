#ifndef __Subimage_hpp__
#define __Subimage_hpp__

#include "opencv2/core.hpp"
#include "utility.h"

struct Subimage
{
  
  Subimage(cv::Point const& tl, cv::Point const& br):
    tl(tl),
    br(br),
    roi_center(),
    value()
    {
      int temp_x = br.x - tl.x;
      int temp_y = br.y - tl.y;
      roi_center = cv::Point(tl.x + (temp_x/2), tl.y + (temp_y/2));
    }

  ~Subimage() {}

  void calculateSubimageValue(cv::Mat const& dMap)
  {
    cv::Rect roi(tl,br);
    cv::Mat temp = dMap(roi);
    value = Utility::calcMeanDisparity(temp);
  }

  // member
  cv::Point   tl;
  cv::Point   br;
  cv::Point   roi_center;
  float       value;
};

#endif //__Subimage_hpp__
