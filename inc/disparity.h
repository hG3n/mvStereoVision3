#ifndef __DISPARITY__H__
#define __DISPARITY__H__

#include "utility.h"

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
// current ly not working
// #include "opencv2/stereo/stereo.hpp"

namespace Disparity
{
  struct sgbmParameters {
    int minDisp;
    int numDisp;
    int blockSize;
    int disp12MaxDiff;
    int preFilterCap;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int disparityMode;
  };

	void sgbm(Stereopair const&, cv::Mat&, cv::Ptr<cv::StereoSGBM>);
  // void binary_sgbm(Stereopair const& , cv::Mat&, cv::Ptr<cv::stereo::StereoBinarySGBM>);
  void bm(Stereopair const&, cv::Mat& , cv::Ptr<cv::StereoBM>);
  void tm(Stereopair const&, cv::Mat& ,unsigned int);

  // parameter loader
  bool loadSGBMParameters(std::string const, cv::Ptr<cv::StereoSGBM>&, sgbmParameters&);
}

#endif //__DISPARITY__H__