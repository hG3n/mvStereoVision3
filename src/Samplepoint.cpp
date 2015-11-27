#include "Samplepoint.h"

Samplepoint::Samplepoint(cv::Point center, int radius): 
  center(center),
  radius(radius),
  roi(),
  value()
{
  cv::Point p1(center.x - radius, center.y - radius);
  cv::Point p2(center.x + radius + 1, center.y + radius + 1);
  roi = cv::Rect(p1,p2);
}

Samplepoint::~Samplepoint()
{}

// -----------------------------------------------------------------------------
// --- calculations ------------------------------------------------------------
// -----------------------------------------------------------------------------
void Samplepoint::calculateSamplepointValue(cv::Mat const& dMap)
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

void Samplepoint::draw(cv::Mat& dMap)
{
  cv::Point p1(center.x - radius, center.y - radius);
  cv::Point p2(center.x + radius + 1, center.y + radius + 1);
  cv::rectangle(dMap, p1, p2, cv::Scalar(255,0,0));
}