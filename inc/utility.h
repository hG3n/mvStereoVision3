#ifndef __UTILITY__H__
#define __UTILITY__H__

#include <dirent.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

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
#include "ply.h"

struct Stereopair
{
	Stereopair();
	Stereopair(cv::Mat &, cv::Mat &);
	Stereopair(cv::Mat &&, cv::Mat &&);
	~Stereopair();

	cv::Mat mLeft;
	cv::Mat mRight;
	std::string mTag;
};

struct CameraInit
{
	Camera* left;
	Camera* right;
	bool init;
};

struct dMapValues
{
  float dValue;
  float image_x;
  float image_y;
};

namespace Utility
{
	// directory 
	int 	getFiles (std::string const& dir, std::vector<std::string> &files);
	bool 	directoryExist(std::string const& dirPath);
	bool 	createDirectory(std::string const& dirPath);

	// init cameras
	bool 	initCameras(mvIMPACT::acquire::DeviceManager&, Camera*&, Camera*&);

	// config
	bool 	checkConfig(std::string const&, std::vector<std::string> const&, cv::FileStorage &);

	// helper
	double 	checkSharpness(cv::Mat const&);

  cv::Mat     calcCoordinate(dMapValues, cv::Mat const&);
  float       calcDistance(dMapValues, cv::Mat const&, int);
  dMapValues  calcDMapValues(cv::Mat_<float> const&, cv::Mat const&);
  void        dmap2pcl(std::string const&, cv::Mat const&, cv::Mat const&);

  float                  calcMeanDisparity(cv::Mat const&);
  std::pair<short,short> calcMinMaxDisparity(cv::Mat const&);
  float									 calcStdDev(cv::Mat const&);

  void subdivideImage(cv::Mat const&, int, std::vector<cv::Mat>&);

  std::string type2str(cv::Mat const&);

  float calcMagnitude(cv::Mat const &);
  float calcAngle(cv::Mat&, cv::Mat&);
}

#endif //__UTILITY__H__
