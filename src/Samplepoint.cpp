#include "Samplepoint.h"

Samplepoint::Samplepoint(cv::Point center, int radius): 
  center(center),
  radius(radius),
  value()
{}

Samplepoint::~Samplepoint()
{}

// -----------------------------------------------------------------------------
// --- calculations ------------------------------------------------------------
// -----------------------------------------------------------------------------
void Samplepoint::calculateSamplepointValue(cv::Mat & dMap)
{ 
  if(radius > 0)
  {

    cv::Point p1(center.x - radius, center.y - radius);
    cv::Point p2(center.x + radius + 1, center.y + radius + 1);
    
    cv::Rect roi(p1,p2);

    cv::rectangle(dMap, p1, p2, cv::Scalar(255,0,0));

    cv::Mat temp = dMap(roi);
    value = Utility::calcMeanDisparity(temp);
  } 
  else
  {
    value = dMap.at<short>(center.x, center.y);
  }
}