#include "SamplepointDetection.h"

SamplepointDetection::SamplepointDetection():
  mTag("SAMPLEPOINT DETECTION\t"),
  mSPVec()
{}

SamplepointDetection::~SamplepointDetection()
{}

// -----------------------------------------------------------------------------
// --- functions ---------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::init(cv::Mat const& reference, cv::Mat const& Q) {
  
  // clear created samplepoints
  mSPVec.clear();

  // create new samplepoints
  int distanceX = reference.cols/8;
  int distanceY = reference.rows/8;

  for (int c = 1; c < distanceX; ++c) {
    for (int r = 1; r < distanceY; ++r) { 
      mSPVec.push_back(Samplepoint(cv::Point(c*(reference.cols/distanceX), r*(reference.rows/distanceY)), 1));
    }
  }

  // set Q member variable
  mQ_32F = Q;

  // center point of the image with z in 'inf'
  mCenterPoint = cv::Mat_<float>(1,4);
  mCenterPoint(0) = reference.cols/2;
  mCenterPoint(1) = reference.rows/2;
  mCenterPoint(2) = 10000;
  mCenterPoint(3) = 0;
}

// -----------------------------------------------------------------------------
// --- getter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::vector<Samplepoint> SamplepointDetection::getSamplepointVec() const 
{
  return mSPVec;
}

cv::Mat_<float> SamplepointDetection::getCenterPoint() const 
{
  return mCenterPoint;
}

// -----------------------------------------------------------------------------
// --- builder -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::build(cv::Mat const& dMap, int binning, int mode)
{
  for(unsigned int i = 0; i < mSPVec.size(); ++i) {
    mSPVec[i].calculateSamplepointValue(dMap);
  }
}

// -----------------------------------------------------------------------------
// --- analyzer ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::detectObstacles() 
{
  float value = mSPVec[500].value;
  cv::Mat_<float> temp = Utility::calcCoordinate(mQ_32F, value, mSPVec[500].center.x, mSPVec[500].center.y);
#define N 32
  std::cout << mCenterPoint << std::endl;
  std::cout << temp.t() << std::endl;
#undef N
  float angle = Utility::calcAngle(mCenterPoint,temp.t());
  std::cout << angle << std::endl;
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
/*virtual*/ void SamplepointDetection::print_on(std::ostream& out) const {
  out << mTag 
      << "\nnumber of Samplepoints: " << mSPVec.size() << " ";
}
