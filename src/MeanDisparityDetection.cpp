#include "MeanDisparityDetection.h"
#include "utility.h"

MeanDisparityDetection::MeanDisparityDetection():
  mTag("MEAN DISPARITY DETECTION\t"),
  mPositions(),
  mMeanMap(),
  mMeanDistanceMap(),
  mQ_32F()
{
  // reserve memory for distance maps
  mMeanDistanceMap.reserve(81);

  // map containing position strings for every image in the final distance map
  mPositions = {{0, "TOP LEFT - 0"}, {1, "TOP LEFT - 1"}, {2, "TOP LEFT - 2"},
                {3, "TOP LEFT - 3"}, {4, "TOP LEFT - 4"}, {5, "TOP LEFT - 5"},
                {6, "TOP LEFT - 6"}, {7, "TOP LEFT - 7"}, {8, "TOP LEFT - 8"},

                {9, "TOP - 0"}, {11, "TOP - 1"}, {12, "TOP - 2"},
                {13, "TOP - 3"}, {14, "TOP - 4"}, {15, "TOP - 5"},
                {16, "TOP - 6"}, {17, "TOP - 7"}, {18, "TOP - 8"},

                {19, "TOP RIGHT - 0"}, {20, "TOP RIGHT - 1"}, {21, "TOP RIGHT - 2"},
                {22, "TOP RIGHT - 3"}, {23, "TOP RIGHT - 4"}, {24, "TOP RIGHT - 5"},
                {25, "TOP RIGHT - 6"}, {26, "TOP RIGHT - 7"}, {27, "TOP RIGHT - 8"},

                {28, "LEFT - 0"}, {29, "LEFT - 1"}, {30, "LEFT - 2"},
                {31, "LEFT - 3"}, {32, "LEFT - 4"}, {33, "LEFT - 5"},
                {34, "LEFT - 6"}, {35, "LEFT - 7"}, {36, "LEFT - 8"},

                {37, "CENTER - 0"}, {38, "CENTER - 1"}, {39, "CENTER - 2"},
                {40, "CENTER - 3"}, {41, "CENTER - 4"}, {42, "CENTER - 5"},
                {43, "CENTER - 6"}, {44, "CENTER - 7"}, {45, "CENTER - 8"},

                {46, "RIGHT - 0"}, {47, "RIGHT - 1"}, {48, "RIGHT - 2"},
                {49, "RIGHT - 3"}, {50, "RIGHT - 4"}, {51, "RIGHT - 5"},
                {52, "RIGHT - 6"}, {53, "RIGHT - 7"}, {54, "RIGHT - 8"},

                {55, "BOTTOM LEFT - 0"}, {56, "BOTTOM LEFT - 1"}, {57, "BOTTOM LEFT - 2"},
                {58, "BOTTOM LEFT - 3"}, {59, "BOTTOM LEFT - 4"}, {60, "BOTTOM LEFT - 5"},
                {61, "BOTTOM LEFT - 6"}, {62, "BOTTOM LEFT - 7"}, {63, "BOTTOM LEFT - 8"},

                {64, "BOTTOM - 0"}, {65, "BOTTOM - 1"}, {66, "BOTTOM - 2"},
                {67, "BOTTOM - 3"}, {68, "BOTTOM - 4"}, {69, "BOTTOM - 5"},
                {70, "BOTTOM - 6"}, {71, "BOTTOM - 7"}, {72, "BOTTOM - 8"},

                {73, "BOTTOM RIGHT - 0"}, {74, "BOTTOM RIGHT - 1"}, {75, "BOTTOM RIGHT - 2"},
                {76, "BOTTOM RIGHT - 3"}, {77, "BOTTOM RIGHT - 4"}, {78, "BOTTOM RIGHT - 5"},
                {79, "BOTTOM RIGHT - 6"}, {80, "BOTTOM RIGHT - 7"}, {81, "BOTTOM RIGHT - 8"}};

  LOG(INFO)<< mTag <<"Obstacle Detection created\n";
}

MeanDisparityDetection::~MeanDisparityDetection()
{}


// -----------------------------------------------------------------------------
// --- init --------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MeanDisparityDetection::init(cv::Mat const& Q)
{
  mQ_32F = Q;
}

// -----------------------------------------------------------------------------
// --- getter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::vector<float> MeanDisparityDetection::getMeanMap() const
{
  return mMeanMap;
}

std::vector<float> MeanDisparityDetection::getMeanDistanceMap() const
{
  return mMeanDistanceMap;
}


// -----------------------------------------------------------------------------
// --- builder -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void MeanDisparityDetection::build(cv::Mat const& dMap, int binning, int mode)
{
  // subdivide dMap for every frame
  std::vector<std::vector<cv::Mat>>  subimages;
  std::vector<cv::Mat> temp;
  Utility::subdivideImage(dMap, binning, temp);

  for (unsigned int i = 0; i < temp.size(); ++i) {
    std::vector<cv::Mat> temp2;
    Utility::subdivideImage(temp[i], binning, temp2);
    subimages.push_back(temp2);
  }
  
  switch(mode) {
    case MODE::MEAN_DISTANCE:
    {
      mMeanDistanceMap.clear();
      unsigned int numSubimages = subimages.size();
      if (numSubimages == 0)
        LOG(INFO)<< mTag <<"Unable to build Mean-Map. No Subimages provided\n";

      for (unsigned int i = 0; i < numSubimages; ++i) {
        for (unsigned int j = 0; j < numSubimages; ++j) {
          float meanValue = Utility::calcMeanDisparity(subimages[i][j]);
          mMeanDistanceMap.push_back(Utility::calcDistance(mQ_32F,meanValue,binning));
        }
      }
      break;
    }

    case MODE::MEAN_VALUE:
    {
      mMeanMap.clear();
      unsigned int numSubimages = subimages.size();
      if (numSubimages == 0)
        LOG(INFO)<< mTag <<"Unable to build Mean-Map. No Subimages provided\n";

      for (unsigned int i = 0; i < numSubimages; ++i) {
        for (unsigned int j = 0; j < numSubimages; ++j) {
          float meanValue = Utility::calcMeanDisparity(subimages[i][j]);
          mMeanMap.push_back(meanValue);
        }
      }
    }
 
  }

}

// -----------------------------------------------------------------------------
// --- analyzer ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void MeanDisparityDetection::detectObstacles()
{
  // analyzes each element in the mean vector, printing location of found obstacles
  int i = 0;
  auto range = mRange;  // seems like it is not possible to directly pass the member to the lambda
  std::for_each(mMeanDistanceMap.begin(), mMeanDistanceMap.end(), [range, &i, this](float value) {
    if(value >= range.first && value <= range.second) {
      std::cout << "obstacle in: " << mPositions[i] << std::endl;
    }
    ++i;
  });
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
/*virtual*/ void MeanDisparityDetection::print_on(std::ostream& out) const {
  out << mTag 
      << "\nmean distance map length: " << mMeanDistanceMap.size() 
      << "\nqMatrix:                  " << mQ_32F.size() << " ";
}
