#include <iostream>
#include <string>
#include <chrono>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"
#include "view.h"
#include "Samplepoint.h"

#include <thread>
#include <mutex>
#include <condition_variable>

INITIALIZE_EASYLOGGINGPP
std::string tag = " Main ";

cv::Ptr<cv::StereoSGBM> disparitySGBM;

int minDisp, numDisp, blockSize, speckleWindowSize, speckleRange, disp12MaxDiff, preFilterCap, uniquenessRatio;
int disparityMode;
bool newDisparityMap = false;
bool reload = false;
int view;

cv::Mat dMapRaw;
cv::Mat dMapNorm;
cv::Mat dMapWork(480, 752, CV_32F);

cv::Mat Q, Q_32F;
cv::Mat R, R_32F;
cv::Mat disparityMap_32FC1;

int main(int argc, char const *argv[])
{

  

  return 0;
}