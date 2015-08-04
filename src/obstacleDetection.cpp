#include "obstacleDetection.h"
#include "utility.h"

obstacleDetection::obstacleDetection():
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

std::vector<float> obstacleDetection::getDistanceMapStdDev() const
{
  return mDistanceMapStdDev;
}


// -----------------------------------------------------------------------------
// --- builder -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void obstacleDetection::buildMeanMap(cv::Mat const& Q, std::vector<std::vector<cv::Mat>> const& subimages)
{
  mMeanMap.clear();
  unsigned int numSubimages = subimages.size();
  if (numSubimages == 0)
    LOG(INFO)<< mTag <<"Unable to build Mean-Map. No Subimages provided\n";

  for (unsigned int i = 0; i < numSubimages; ++i)
  {
    for (unsigned int j = 0; j < numSubimages; ++j)
    {
      float meanValue = Utility::calcMeanDisparity(subimages[i][j]);
      mMeanMap.push_back(meanValue);
    }
  }
}

void obstacleDetection::buildMeanDistanceMap(cv::Mat const& Q, std::vector<std::vector<cv::Mat>> const& subimages, int binning)
{
  mDistanceMapMean.clear();
  unsigned int numSubimages = subimages.size();
  if (numSubimages == 0)
    LOG(INFO)<< mTag <<"Unable to build Mean-Map. No Subimages provided\n";

  for (unsigned int i = 0; i < numSubimages; ++i)
  {
    for (unsigned int j = 0; j < numSubimages; ++j)
    {
      float meanValue = Utility::calcMeanDisparity(subimages[i][j]);
      mDistanceMapMean.push_back(Utility::calcDistance(Q,meanValue,binning));
    }
  }
}

void obstacleDetection::buildStdDevDistanceMap(cv::Mat const& Q, std::vector<std::vector<cv::Mat>> const& subimages)
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
    
    

}
