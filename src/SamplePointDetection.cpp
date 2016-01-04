#include "SamplepointDetection.h"

SamplepointDetection::SamplepointDetection():
  ObstacleDetection(),
  mTag("SAMPLEPOINT DETECTION\t")
{}

SamplepointDetection::~SamplepointDetection()
{}

// -----------------------------------------------------------------------------
// --- functions ---------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::init(cv::Mat const& reference, cv::Mat const& Q, float min_distance, float max_distance) {
  
  // clear created samplepoints
  mSPVec.clear();

  // set Q member variable
  mQ_32F = Q;

  // create new samplepoints
  int distanceX = reference.cols/8;
  int distanceY = reference.rows/8;

  for (int c = 1; c < distanceX; ++c) {
    for (int r = 1; r < distanceY; ++r) { 
      mSPVec.push_back(Samplepoint(cv::Point(c*(reference.cols/distanceX), r*(reference.rows/distanceY)), 2));
    }
  }

  // center point of the image with z in 
  mCenterVec = cv::Mat_<float>(1,4);
  mCenterVec(0) = 0.0f;
  mCenterVec(1) = 0.0f;
  mCenterVec(2) = 1.0f;
  mCenterVec(3) = 0.0f;

  // set image center point according to principal point given in mQ_32F
  mImageCenter.x = mQ_32F.at<float>(0,3);
  mImageCenter.y = mQ_32F.at<float>(1,3);
 
  // create empty pointcloud
  mPointcloud = cv::Mat::zeros(reference.rows, reference.cols, CV_32F);

  // intialize disparityRange
  cv::Mat_<float> lower(1,3);
  lower(0) = 0;
  lower(1) = 0;
  lower(2) = min_distance * 1000;

  cv::Mat_<float> upper(1,3);
  upper(0) = 0;
  upper(1) = 0;
  upper(2) = max_distance * 1000;

  dMapValues d_lower = Utility::calcDMapValues(lower, mQ_32F);
  dMapValues d_upper = Utility::calcDMapValues(upper, mQ_32F);

  mRangeDisparity = std::make_pair(d_lower.dValue, d_upper.dValue);
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

std::pair<int,int> SamplepointDetection::getRange() const
{
  return mRange;
}

std::vector<Samplepoint> SamplepointDetection::getFoundObstacles() const
{
  return mFoundObstacles;
}


// -----------------------------------------------------------------------------
// --- setter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SamplepointDetection::setRange(float min_distance, float max_distance)
{
  mRange.first = min_distance;
  mRange.second = max_distance;
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
  // TODO: just use points in a specific range
  std::vector<unsigned int> distance_indices;
  for(unsigned int i = 0; i < mSPVec.size(); ++i) {
      if(mSPVec[i].value > mRange.first && mSPVec[i].value < mRange.second) {
        // store the index where an obstacle was found
        distance_indices.push_back(i);
      }
  }

  // calculate the distances of all sp within the  given range
  // store ids and distances in 
  std::vector<std::pair<int,float>> distance_storage;
  for(unsigned int i = 0; i < distance_indices.size(); ++i) {
    cv::Point current_center = mSPVec[distance_indices[i]].center;
    float distance = sqrt(pow(current_center.x - mImageCenter.x,2 ) + pow(current_center.y - mImageCenter.y,2));
    distance_storage.push_back(std::make_pair(distance_indices[i], distance));
  }

  std::sort(distance_storage.begin(), distance_storage.end(), SamplepointDetection::sort_distances());

  for (unsigned int i = 0; i < distance_storage.size(); ++i)
  {
    std::cout << "id: " <<  distance_storage[i].first << " distance: " << distance_storage[i].second << std::endl;
  }
  std::cout << std::endl;


  // reasonable to add distance to origin to samplepoint struct???
  // save index of variable in order to identify the nearest points

  /*  std::cout << distance_indices.size() << std::endl;
    for(unsigned int i = 0; i < distance_indices.size(); ++i) {
      std::cout << mSPVec[distance_indices[i]].center << std::endl;
    }
  */
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
