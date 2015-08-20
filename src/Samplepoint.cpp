#include "Samplepoint.h"

Samplepoint::Samplepoint(cv::Point center, int radius): 
  center{center},
  radius{radius},
  value{}
{}

Samplepoint::~Samplepoint()
{}

// -----------------------------------------------------------------------------
// --- calculations ------------------------------------------------------------
// -----------------------------------------------------------------------------
void Samplepoint::calculateSamplepointValue(cv::Mat const& dMap)
{ 
  if(radius > 0)
  {
    cv::Rect roi(cv::Point(center.x - radius, center.y - radius),
                 cv::Point(center.x + radius, center.y + radius));
    cv::Mat temp = dMap(roi);
    value = Utility::calcMeanDisparity(temp);
  } 
  else
  {
    value = dMap.at<short>(center.y, center.x);
  }
}