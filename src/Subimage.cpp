#include <Subimage.h>

Subimage::Subimage(cv::Point const& tl, cv::Point const& br):
  tl(tl),
  br(br),
  roi_center(),
  value()
{
  int temp_x = br.x - tl.x;
  int temp_y = br.y - tl.y;
  roi_center = cv::Point(temp_x, temp_y);
}

Subimage::~Subimage()
{}

// -----------------------------------------------------------------------------
// --- calculations ------------------------------------------------------------
// -----------------------------------------------------------------------------
void Subimage::calculateSubimageValue(cv::Mat const& dMap)
{
  cv::Rect roi(tl,br);
  cv::Mat temp = dMap(roi);
  value = Utility::calcMeanDisparity(temp);
} 