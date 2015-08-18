#ifndef __OBSTACLE_DETECION__H
#define __OBSTACLE_DETECION__H

#include <vector>
#include <iostream>

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class obstacleDetection
{
  public:

  // mode enum
  enum MODE {MEAN_DISTANCE, SAMPLEPOINTS};
  // std::map<int,std::string> positions = {0, ""};


  obstacleDetection();
  obstacleDetection(cv::Mat const&, int);
  ~obstacleDetection();

  // getter
  std::vector<float>    getMeanMap() const;
  std::vector<float>    getDistanceMapMean() const;
  std::vector<float>    getDistanceMapStdDev() const;

  // builder
  std::vector<cv::Mat> subdivideImage();
  void buildMeanMap(cv::Mat const&, std::vector<std::vector<cv::Mat>> const&);
  void buildMeanDistanceMap(cv::Mat const&, std::vector<std::vector<cv::Mat>> const&, int);
  void buildStdDevDistanceMap(cv::Mat const&, std::vector<std::vector<cv::Mat>> const&);

  // analyzer
  void detectObstacles(int const&, std::pair<float,float> const&);

  private:
  std::string                 mTag;
  std::vector<cv::Point>      mSamplepoints;
  std::vector<float>          mDistanceMapMean;
  std::vector<float>          mDistanceMapMin;
  std::vector<float>          mDistanceMapStdDev;
  std::vector<float>          mMeanMap;
};

#endif //__OBSTACLE_DETECION__H