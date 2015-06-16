#include <iostream>
#include <string>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"
#include "obstacleDetection.h"
#include "view.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#define MEAN_DISTANCE 0
#define MIN_DISTANCE 1
#define STDDEV 2
#define SAMPLE 3

INITIALIZE_EASYLOGGINGPP
bool running = true;

// threading stuff
std::mutex disparityLock;
std::condition_variable cond_var;

cv::Ptr<cv::StereoSGBM> disparitySGBM;
int numDisp = 64;
int windSize = 9;
bool newDisparityMap = false;

cv::Mat dMapRaw;
cv::Mat dMapNorm;

cv::Mat Q, Q_32F;
cv::Mat R, R_32F;
cv::Mat disparityMap_32FC1;

void disparityCalc(Stereopair const& s, cv::Ptr<cv::StereoSGBM> disparity)
{
  while(running)
  {
    std::unique_lock<std::mutex> ul(disparityLock);
    cond_var.wait(ul);
    Disparity::sgbm(s, dMapRaw, disparity);
    dMapRaw.convertTo(disparityMap_32FC1,CV_32FC1);
    newDisparityMap=true;
  }
}

void changeNumDispSGBM(int, void*)
{
  numDisp+=numDisp%16;

  if(numDisp < 16)
  {
      numDisp = 16;
      cv::setTrackbarPos("Num Disp", "SGBM", numDisp);
  }

  cv::setTrackbarPos("Num Disp", "SGBM", numDisp);
  disparitySGBM->setNumDisparities(numDisp);
}

void changeWindSizeSGBM(int, void*)
{
  if(windSize%2 == 0)
      windSize+=1;

  if(windSize < 5)
  {
      windSize = 5;
      cv::setTrackbarPos("Wind Size", "SGBM", windSize);
  }
  cv::setTrackbarPos("Wind Size", "SGBM", windSize);
  disparitySGBM->setBlockSize(windSize);
}


void mouseClick(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN )
  {
    double d = static_cast<float>(dMapRaw.at<short>(y,x));
    float distance = Utility::calcDistance(Q_32F, d, 1);
    std::cout << "disparityValue: " << d << "  distance: " << distance << std::endl;
  }
}


void initWindows()
{
  cv::namedWindow("SGBM" ,1);
  cv::createTrackbar("Num Disp", "SGBM", &numDisp, 320, changeNumDispSGBM);
  cv::createTrackbar("Wind Size", "SGBM", &windSize, 51, changeWindSizeSGBM);
  cv::setMouseCallback("SGBM", mouseClick, NULL);
}


int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started." << std::endl;
  mvIMPACT::acquire::DeviceManager devMgr;

  // create both cameras and set exposure mode to auto
  Camera *left;
  Camera *right;

  if(!Utility::initCameras(devMgr,left,right))
    return 0;

  Stereosystem stereo(left,right);

  left->setExposureMode(1);
  right->setExposureMode(1);
  left->setRequestTimeout(400);
  right->setRequestTimeout(400);

  // load settings from file and check for completeness
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

  if(!stereo.loadIntrinsic(inputParameter+"/intrinsic.yml"))
    return 0;
  if(!stereo.loadExtrinisic(inputParameter +"/extrinsic.yml"))
    return 0;

  // create stereo image pair
  Stereopair s;

  char key = 0;
  int binning = 0;

  stereo.getRectifiedImagepair(s);
  Q = stereo.getQMatrix();
  Q.convertTo(Q_32F,CV_32F);

  cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  initWindows();

  disparitySGBM = cv::StereoSGBM::create(0,numDisp,windSize,8*windSize*windSize,32*windSize*windSize);
  std::thread disparity(disparityCalc,std::ref(s),std::ref(disparitySGBM));

  std::vector<cv::Mat> subimages;
  // obstacleDetection obst(dMapRaw, binning);

  running = true;
  int frame = 0;  
  while(running)
  {

    stereo.getRectifiedImagepair(s);
    cv::imshow("Left", s.mLeft);
    cv::imshow("Right", s.mRight);

    // mean map storage
    std::vector<std::vector<float>> v;

    if(newDisparityMap)
    {
      Utility::subdivideImage(dMapRaw, binning, subimages);

      // obst.buildMeanDistanceMap(Q_32F);
      // v = obst.getDistanceMapMean();
      if (binning == 0)
      {
        // obst.detectObstacles(MEAN_DISTANCE, std::make_pair(1.2,2.0));
        // obst.detectObstacles(MIN_DISTANCE, std::make_pair(1.2,2.0));
      }
      else
      {
        // obst.detectObstacles(MEAN_DISTANCE, std::make_pair(0.8,1.2));
        // obst.detectObstacles(MIN_DISTANCE, std::make_pair(0.8,1.2));
      }

      // display stuff
      cv::normalize(dMapRaw,dMapNorm,0,255,cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(dMapNorm,dMapNorm,CV_GRAY2BGR);
      View::drawObstacleGrid(dMapNorm, binning);
      View::drawSubimageGrid(dMapNorm, binning);
      cv::imshow("SGBM",dMapNorm);
      newDisparityMap = false;
    }

    // notify the thread to start 
    cond_var.notify_one();
    key = cv::waitKey(5);

    // keypress stuff
    if(key > 0)
    {
      switch(key)
      {
        case 'q':
          LOG(INFO) << tag << "Exit requested" <<std::endl;
          delete left;
          left = nullptr;
          delete right;
          right = nullptr;
          cond_var.notify_one();
          running = false;
          break;
        case 'b':
          if (binning == 0)
            binning = 1;
          else
            binning = 0;
          left->setBinning(binning);
          right->setBinning(binning);
          stereo.resetRectification();
          break;
        case 'f':
          std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;
          break;
        default:
          std::cout << "Key pressed has no action" <<std::endl;
          break;
      }
    }
    ++frame;
  }

  disparity.join();
  return 0;
}
