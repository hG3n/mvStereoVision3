#include <iostream>
#include <string>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"

#include <thread>
#include <mutex>
#include <condition_variable>

INITIALIZE_EASYLOGGINGPP

bool running = true;
cv::Mat dispMapSGBM;

cv::Ptr<cv::StereoSGBM> disparitySGBM;

int numDisp = 64;
int windSize = 9;

cv::Mat R, Q, R_32F, Q_32F;

int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started.";
  mvIMPACT::acquire::DeviceManager devMgr;

  Camera *left;
  Camera *right;

  if(!Utility::initCameras(devMgr,left,right))
    return 0;

  left->setExposureMode(1);
  right->setExposureMode(1);

  Stereopair s;

  std::vector<std::string> nodes;
  nodes.push_back("inputParameter");
  std::string config = "./configs/default.yml";
  cv::FileStorage fs;
  if(!Utility::checkConfig(config,nodes,fs))
    return 0;

  std::string inputParameter;
  fs["inputParameter"] >> inputParameter;
  std::string outputDirectory;
  fs["capturedDisparities"] >> outputDirectory;

  Stereosystem stereo(left,right);

  if(!stereo.loadIntrinsic(inputParameter+"/intrinsic.yml"))
    return 0;
  if(!stereo.loadExtrinisic(inputParameter +"/extrinsic.yml"))
    return 0;

  disparitySGBM = cv::StereoSGBM::create(0,numDisp,windSize,8*windSize*windSize,32*windSize*windSize);

  stereo.getRectifiedImagepair(s);

  R = stereo.getRotationMatrix();
  Q = stereo.getQMatrix();

  R.convertTo(R_32F,CV_32F);
  Q.convertTo(Q_32F,CV_32F);

  std::cout << Q_32F << std::endl;

  int key = 0;
  int binning =0;

  cv::Mat normalizedSGBM;
  cv::Mat leftColor;

  cv::namedWindow("Left");
  cv::namedWindow("Right");
  cv::namedWindow("SGBM");
  while(running)
  {
    stereo.getRectifiedImagepair(s);

    cvtColor(s.mLeft, leftColor, CV_GRAY2RGB);

    cv::imshow("Left", leftColor);
    cv::imshow("Right", s.mRight);

    disparitySGBM->compute(s.mLeft, s.mRight, dispMapSGBM);
    cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
    cv::imshow("SGBM",normalizedSGBM);

    key = cv::waitKey(10);

    if(char(key) == 'q')
      running = false;
    else if(char(key) == 'f')
      std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;
  }
  return 0;
}
