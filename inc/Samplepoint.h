#ifndef __Samplepoint_hpp__
#define __Samplepoint_hpp__

#include "opencv2/core.hpp"
#include "utility.h"

struct Samplepoint
{
  Samplepoint(cv::Point center, int radius): 
    center(center),
    radius(radius),
    roi(),
    value()
    {
      cv::Point p1(center.x - radius, center.y - radius);
      cv::Point p2(center.x + radius + 1, center.y + radius + 1);
      roi = cv::Rect(p1,p2);
    }

  ~Samplepoint()
  {}

  // methods
  void calculateSamplepointValue(cv::Mat const& dMap)
  { 
    if(radius > 0)
    {
      cv::Mat temp = dMap(roi);
      value = Utility::calcMeanDisparity(temp);
      // std::cout << value << std::endl;
    } 
    else
    {
      value = dMap.at<short>(center.x, center.y);
    }
  }

  // draws the samplepoint
  void draw(cv::Mat& dMap)
  {
    cv::Point p1(center.x - radius, center.y - radius);
    cv::Point p2(center.x + radius + 1, center.y + radius + 1);
    cv::rectangle(dMap, p1, p2, cv::Scalar(255,0,0));
  }

  // member
  cv::Point   center;
  int         radius;
  cv::Rect    roi;
  float       value;
};

#endif //__Samplepoint_hpp__
