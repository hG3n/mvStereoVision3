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
  mCenterVec = cv::Mat_<float>(1,4);
  mCenterVec(0) = 0.0f;
  mCenterVec(1) = 0.0f;
  mCenterVec(2) = 1.0f;
  mCenterVec(3) = 0.0f;

  // set image center point according to principal point given in mQ_32F
  mImageCenter.x = mQ_32F.at<float>(0,3);
  mImageCenter.y = mQ_32F.at<float>(1,3);
 
  std::cout << mImageCenter << std::endl;
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
  return mCenterVec;
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
  // float value = mSPVec[500].value;
  // cv::Mat_<float> temp = Utility::calcCoordinate(mQ_32F, value, mSPVec[500].center.x, mSPVec[500].center.y);

  // calculate the distance of each samplepoint to the image center point
  // TODO: just use points in a specific range
  std::vector<float> distances_2d;
  for(unsigned int i = 0; i < mSPVec.size(); ++i) {
    /*
      if(mSPVec[i].value > mRange.first && mSPVec[i].value < mRange.second) {
        float distance = sqrt( pow(mImageCenter.x - mSPVec[i].center.x, 2) + pow(mImageCenter.y = mSPVec[i].center.y,2));
        push samplepoints in new detection vector 
        reasonable to add  distance to origin to samplepoint struct???

      }
    */
    float temp = sqrt( pow(mImageCenter.x - mSPVec[i].center.x, 2) + pow(mImageCenter.y = mSPVec[i].center.y,2));
    distances_2d.push_back(temp);
  }

  /* 
    sort detection vector by distance and disparity value 
    calculate angle between lowest 2 and center vector
    return minimal angles (x and z) and distance of nearest 2 obstacles
  */


  // float angle = Utility::calcAngle(mCenterVec,temp);
  // std::cout << angle << std::endl;
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
/*virtual*/ void SamplepointDetection::print_on(std::ostream& out) const {
  out << mTag 
      << "\nnumber of Samplepoints: " << mSPVec.size() << " ";
}
