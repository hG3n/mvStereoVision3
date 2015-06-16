#include "obstacleDetection.h"
#include "utility.h"

obstacleDetection::obstacleDetection():
  mTag("OBSTACLE DETECTION\t"),
  mSamplepoints(),
  mDistanceMapMean(),
  mDistanceMapMin(),
  mDistanceMapStdDev(),
  mMeanMap()
{}

obstacleDetection::obstacleDetection(cv::Mat const& disparityMap, int binning):
  mTag("OBSTACLE DETECTION\t"),
  mSamplepoints(),
  mDistanceMapMean(),
  mDistanceMapMin(),
  mDistanceMapStdDev(),
  mMeanMap()
{
  // reserve memory for distance maps
  mDistanceMapMin.reserve(81);
  mDistanceMapMean.reserve(81);
  mDistanceMapStdDev.reserve(81);

  LOG(INFO)<< mTag <<"Obstacle Detection created\n";
}

obstacleDetection::~obstacleDetection()
{}


// -----------------------------------------------------------------------------
// --- getter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::vector<float> obstacleDetection::getMeanMap() const
{
  return mMeanMap;
}

std::vector<float> obstacleDetection::getDistanceMapMean() const
{
  return mDistanceMapMean;
}

std::vector<float> obstacleDetection::getDistanceMapMin() const
{
  return mDistanceMapMin;
}

std::vector<float> obstacleDetection::getDistanceMapStdDev() const
{
  return mDistanceMapStdDev;
}


// -----------------------------------------------------------------------------
// --- builder -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void obstacleDetection::buildMeanMap(cv::Mat const& Q, std::vector<cv::Mat> const& subimages)
{
  mMeanMap.clear();
  unsigned int numSubimages = subimages.size();
  if (numSubimages == 0)
    LOG(INFO)<< mTag <<"Unable to build Mean-Map. No Subimages provided\n";

  for (unsigned int i = 0; i < subimages.size(); ++i)
  {
    float meanValue = Utility::calcMeanDisparity(subimages[i]);
    mMeanMap.push_back(meanValue);
  }

}

void obstacleDetection::buildMeanDistanceMap(cv::Mat const& Q, std::vector<cv::Mat> const& subimages)
{
  mDistanceMapMean.clear();
  unsigned int numSubimages = subimages.size();
  if (numSubimages == 0)
    LOG(INFO)<< mTag <<"Unable to build Mean-Distance-Map. No Subimages provided\n";
}


void obstacleDetection::buildMinDistanceMap(cv::Mat const& Q, std::vector<cv::Mat> const& subimages)
{
  mDistanceMapMin.clear();
  unsigned int numSubimages = subimages.size();
  if (numSubimages == 0)
    LOG(INFO)<< mTag <<"Unable to build Min-Distance-Map. No Subimages provided\n";
}

void obstacleDetection::buildStdDevDistanceMap(cv::Mat const& Q, std::vector<cv::Mat> const& subimages)
{
  mDistanceMapStdDev.clear();
  unsigned int numSubimages = subimages.size();
  if (numSubimages == 0)
    LOG(INFO)<< mTag <<"Unable to build Min-Distance-Map. No Subimages provided\n";
}

// -----------------------------------------------------------------------------
// --- analyzer ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void obstacleDetection::detectObstacles(int const& mode, std::pair<float,float> const& threshold)
{
  std::cout << "foo" << std::endl;
}
