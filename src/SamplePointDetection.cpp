#include "SamplepointDetection.h"

SamplepointDetection::SamplepointDetection():
  mTag("SAMPLEPOINT DETECTION\t"),
  mSPVector()
{}

SamplepointDetection::~SamplepointDetection()
{}

// -----------------------------------------------------------------------------
// --- functions ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::init(cv::Mat const& reference) {
  
  int distanceX = reference.cols/8;
  int distanceY = reference.rows/8;

  for (int c = 1; c < distanceX; ++c) {
    for (int r = 1; r < distanceY; ++r) { 
      mSPVector.push_back(Samplepoint(cv::Point(c*(reference.cols/distanceX), r*(reference.rows/distanceY)), 1));
    }
  }

}

// -----------------------------------------------------------------------------
// --- builder -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::build(cv::Mat const& dMap, int binning, int mode)
{
  std::cout << "Builder called" << std::endl;
}

// -----------------------------------------------------------------------------
// --- analyzer ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::detectObstacles() 
{
  std::cout << "detection called" << std::endl;
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
/*virtual*/ void SamplepointDetection::print_on(std::ostream& out) const {
  out << mTag 
      << "\nnumber of Samplepoints: " << mSPVector.size() << " ";
}
