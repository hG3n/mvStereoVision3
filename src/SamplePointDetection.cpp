#include "SamplepointDetection.h"

SamplepointDetection::SamplepointDetection():
  mTag("SAMPLEPOINT DETECTION\t"),
  mSPVec()
{}

SamplepointDetection::~SamplepointDetection()
{}

// -----------------------------------------------------------------------------
// --- functions -------------------------------------------------------------swa--
// -----------------------------------------------------------------------------
void SamplepointDetection::init(cv::Mat const& reference) {
  
  mSPVec.clear();

  int distanceX = reference.cols/8;
  int distanceY = reference.rows/8;

  for (int c = 1; c < distanceX; ++c) {
    for (int r = 1; r < distanceY; ++r) { 
      mSPVec.push_back(Samplepoint(cv::Point(c*(reference.cols/distanceX), r*(reference.rows/distanceY)), 1));
    }
  }

}

// -----------------------------------------------------------------------------
// --- getter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::vector<Samplepoint> SamplepointDetection::getSamplepointVec() const 
{
  return mSPVec;
}

// -----------------------------------------------------------------------------
// --- builder -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::build(cv::Mat const& dMap, int binning, int mode)
{
  std::for_each(mSPVec.begin(), mSPVec.end(), [&dMap](Samplepoint s){
    s.calculateSamplepointValue(dMap);
  });
}

// -----------------------------------------------------------------------------
// --- analyzer ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::detectObstacles() 
{
  for( auto i = mSPVec.begin(); i < mSPVec.end(); ++i)
  {
    std::cout << (*i).value << std::endl;
  }
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
/*virtual*/ void SamplepointDetection::print_on(std::ostream& out) const {
  out << mTag 
      << "\nnumber of Samplepoints: " << mSPVec.size() << " ";
}
