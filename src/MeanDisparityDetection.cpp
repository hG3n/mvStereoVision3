#include "MeanDisparityDetection.h"
#include "utility.h"

#include <chrono>
#include <sstream>
#include <ctime>

MeanDisparityDetection::MeanDisparityDetection():
  ObstacleDetection(),
  mTag("MEAN DISPARITY DETECTION\t"),
  mPositions(),
  mMeanMap(),
  mMeanDistanceMap(),
  mQ_32F(),
  mPointcloud(),
  mDetectionMode()
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
void MeanDisparityDetection::init(cv::Mat const& reference, cv::Mat const& Q, float min_distance, float max_distance)
{
  // clear existing Subimages
  mSubimageVec.clear();

  // set Q member variable
  mQ_32F = Q;

  // horizontal and vertical distance according to the grade of segementation
  int distanceX = reference.cols/9;
  int distanceY = reference.rows/9;

  // fill the Subimage vector with segmented image
  for (int r = 0; r < 9; ++r) {
    for (int c = 0; c < 9; ++c) {
      int tl_x = c * distanceX;
      int tl_y = r * distanceY;
      int br_x = c * distanceX + distanceX;
      int br_y = r * distanceY + distanceY;
      mSubimageVec.push_back(Subimage(cv::Point(tl_x,tl_y), cv::Point(br_x, br_y)));
    }
  }

  // create empty pointcloud
  mPointcloud = cv::Mat::zeros(reference.rows, reference.cols, CV_32F);

  // initialize disparityRange
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

  LOG(INFO) << mTag << "Disparty Range: (" << d_lower.dValue << " , " << d_upper.dValue << ")" ;
}

// -----------------------------------------------------------------------------
// --- getter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::vector<Subimage> MeanDisparityDetection::getSubimageVec() const
{
  return mSubimageVec;
}

std::vector<float> MeanDisparityDetection::getMeanMap() const
{
  return mMeanMap;
}

std::vector<float> MeanDisparityDetection::getMeanDistanceMap() const
{
  return mMeanDistanceMap;
}

std::pair<int,int> MeanDisparityDetection::getRange() const
{
  return mRange;
}

// -----------------------------------------------------------------------------
// --- setter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MeanDisparityDetection::setRange(float min_distance, float max_distance)
{
  mRange.first = min_distance;
  mRange.second = max_distance;
}

// -----------------------------------------------------------------------------
// --- builder -----------------------------------------------------------------
// -----------------------------------------------------------------------------
void MeanDisparityDetection::build(cv::Mat const& dMap, int binning, int mode)
{
  // create DTO for carrying the disparity values to calcDistance
  dMapValues dMapValues;
  dMapValues.image_x = 0;
  dMapValues.image_y = 0;
  dMapValues.dValue = 0;
  
  switch(mode) {
    case MODE::MEAN_DISTANCE:
    {
      mDetectionMode = MODE::MEAN_DISTANCE;

      // clear previous distance values
      mMeanDistanceMap.clear();

      cv::Mat_<float> temp_Q = mQ_32F;
      std::for_each(mSubimageVec.begin(), mSubimageVec.end(), [this, &dMapValues, dMap, temp_Q, binning] (Subimage s) {
        // calculate mean disparity value for each sample point
        s.calculateSubimageValue(dMap);

        // calculate distance according to the Subimage center
        dMapValues.image_x = s.roi_center.x;
        dMapValues.image_y = s.roi_center.y;
        dMapValues.dValue = s.value;

        // fill mean distance map with distances calculated
        mMeanDistanceMap.push_back(Utility::calcDistance(dMapValues, temp_Q, 0));
      });
    }

    case MODE::MEAN_VALUE:
    {
      mDetectionMode = MODE::MEAN_VALUE;
      // clear previous disparity values
      mMeanMap.clear();

      std::for_each(mSubimageVec.begin(), mSubimageVec.end(), [this, dMap] (Subimage s) {
        // calculate mean disparity value for each sample point
        s.calculateSubimageValue(dMap);

        // fill mean map with median disparity values
        mMeanMap.push_back(s.value);
      });
    }
  }
}

// -----------------------------------------------------------------------------
// --- analyzer ----------------------------------------------------------------
// -----------------------------------------------------------------------------
void MeanDisparityDetection::detectObstacles()
{
  if(mDetectionMode == MODE::MEAN_DISTANCE) {
   
    for(unsigned int i = 0; i < mMeanDistanceMap.size(); ++i) {
      if(mMeanDistanceMap[i] < mRange.second && mMeanDistanceMap[i] > mRange.first) {
        std::cout << "Obstacle found in: " << i << std::endl;
      }
    }

  } else if (mDetectionMode == MODE::MEAN_VALUE) {

    dMapValues dMapValues;
    for(unsigned int i = 0; i < mMeanMap.size(); ++i) {
      std::cout << mMeanMap[i] << std::endl;
      if(mMeanMap[i] < mRangeDisparity.first && mMeanMap[i] > mRangeDisparity.second) {
        std::cout << "Obstacle found in: " << i << std::endl;
        std::cout << mMeanMap[i] << std::endl;
        Subimage temp_s = mSubimageVec[i];

        dMapValues.image_x = temp_s.roi_center.x;
        dMapValues.image_y = temp_s.roi_center.y;
        dMapValues.dValue = temp_s.value;

        // TODO write pointcloud to arbitrary vector
        mPointcloud.at<cv::Vec3f>(temp_s.roi_center.x, temp_s.roi_center.y) =  
          Utility::calcDistance(dMapValues, mQ_32F, 1);
          // Utility::calcCoordinate(mQ_32F, temp_s.value, temp_s.roi_center.x, temp_s.roi_center.y);
      }
    }
    cv::imwrite("pcl/pcl_" + std::to_string(clock()) + ".png", mPointcloud);
  }
}

// -----------------------------------------------------------------------------
// --- operator ----------------------------------------------------------------
// -----------------------------------------------------------------------------
/*virtual*/ void MeanDisparityDetection::print_on(std::ostream& out) const {
  out << mTag 
      << "\nmean distance map length: " << mMeanDistanceMap.size() 
      << "\nqMatrix:                  " << mQ_32F.size() << " ";
}
