//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Camera stuff
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

//Logging stuff
#include "easylogging++.h"

#include "Camera.h"

namespace View
{
  void drawObstacleGrid(cv::Mat &, int);
  void drawSubimageGrid(cv::Mat &, int);
}