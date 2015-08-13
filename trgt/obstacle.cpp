#include <iostream>
#include <string>
#include <chrono>

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
std::string tag = " Main ";
bool running = true;

// threading stuff
std::mutex disparityLock;
std::condition_variable cond_var;

cv::Ptr<cv::StereoSGBM> disparitySGBM;
cv::Ptr<cv::StereoBM> disparityBM;

int minDisp, numDisp, blockSize, speckleWindowSize, speckleRange, disp12MaxDiff, preFilterCap, uniquenessRatio;
int disparityMode;
bool newDisparityMap = false;

cv::Mat dMapRaw;
cv::Mat dMapNorm;

cv::Mat Q, Q_32F;
cv::Mat R, R_32F;
cv::Mat disparityMap_32FC1;

int exposure = 10000;
float gain = 4.0;
bool hdr = false;

void disparityCalcSGBM(Stereopair const& s, cv::Ptr<cv::StereoSGBM> disparity)
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

void mouseClick(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN )
  {
    double d = static_cast<float>(dMapRaw.at<short>(y,x));
    float distance = Utility::calcDistance(Q_32F, d, 1);
    std::cout << "disparityValue: " << d << "  distance: " << distance << std::endl;
    std::cout << "x: " << x << " | y: " << y  << std::endl;
  }
}

bool loadDisparityParameters(std::string const filename)
{
  cv::FileStorage fs;
  bool success = fs.open(filename, cv::FileStorage::READ);

  if(success)
  {
    if(fs["numDisp"].empty() || fs["blockSize"].empty() || fs["speckleWindowSize"].empty() || fs["speckleWindowRange"].empty())    {
      LOG(ERROR) << "Node in " << filename << " is empty" << std::endl;
      fs.release();
      return false;
    }

    fs["minDisp"]        >> minDisp;
    fs["numDisp"]      >> numDisp;
    fs["blockSize"]           >> blockSize;
    fs["disp12MaxDiff"]       >> disp12MaxDiff;
    fs["preFilterCap"]        >> preFilterCap;
    fs["uniquenessRatio"]     >> uniquenessRatio;
    fs["speckleWindowSize"]   >> speckleWindowSize;
    fs["speckleWindowRange"]  >> speckleRange;
    fs["mode"]                >> disparityMode;

    disparitySGBM->setMinDisparity(minDisp);
    disparitySGBM->setNumDisparities(numDisp);
    disparitySGBM->setBlockSize(blockSize);
    disparitySGBM->setPreFilterCap(preFilterCap);
    disparitySGBM->setUniquenessRatio(uniquenessRatio);
    disparitySGBM->setDisp12MaxDiff(disp12MaxDiff);
    disparitySGBM->setSpeckleWindowSize(speckleWindowSize);
    disparitySGBM->setSpeckleRange(speckleRange);

    if(disparityMode == 1 )
      disparitySGBM->setMode(cv::StereoSGBM::MODE_HH);
    else
      disparitySGBM->setMode(cv::StereoSGBM::MODE_SGBM);
  
    LOG(INFO) << "Successfully loaded disparity parameters" << std::endl;

    fs.release();
    return true;
  }
  else
  {
    LOG(ERROR) << "Unable to open disparity parameters" << std::endl;
    fs.release();
    return false;
  }
}

void subdivideImages(cv::Mat const& dMap, std::vector<std::vector<cv::Mat>> &subimages, int binning)
{
  // build a vector containing 81 elements
  // the elements are ordererd along to a more coarser 3x3 grid
  // after subdividing has been completed the images are ordered
  // by 9 elements for every coarser 'subimage'
  std::vector<cv::Mat> temp;
  Utility::subdivideImage(dMapRaw, binning, temp);

  for (unsigned int i = 0; i < temp.size(); ++i)
  {
    std::vector<cv::Mat> temp2;
    Utility::subdivideImage(temp[i], binning, temp2);
    subimages.push_back(temp2);
  }
}

int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started." << std::endl;
  mvIMPACT::acquire::DeviceManager devMgr;

  // create both cameras
  Camera *left;
  Camera *right;

  // initialize cameras and create Stereosystem
  if(!Utility::initCameras(devMgr,left,right))
    return 0;

  Stereosystem stereo(left,right);

  // reset intial exposure and request timeout in ms
  left->setExposure(exposure);
  right->setExposure(exposure);
  left->setRequestTimeout(2000);
  right->setRequestTimeout(2000);

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
  // Stereopair s{cv::Mat{left->getImageHeight(), left->getImageWidth(), CV_8UC1, cv::Scalar::all(0)},
                // cv::Mat{right->getImageHeight(), right->getImageWidth(), CV_8UC1, cv::Scalar::all(0)}};
  Stereopair s;

  char key = 0;
  int binning = 0;

  // get one single imagepair in order to receive the Q Matrix
  // which is created during the rectification
  stereo.getRectifiedImagepair(s);
  Q = stereo.getQMatrix();
  Q.convertTo(Q_32F,CV_32F);

  // init OpenCV GUI objects
  cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("SGBM" ,1);
  cv::setMouseCallback("SGBM", mouseClick, NULL);

  // create dispparity object and connected thread
  disparitySGBM = cv::StereoSGBM::create(minDisp,
                                         numDisp,
                                         blockSize,
                                         8*blockSize*blockSize,
                                         32*blockSize*blockSize);
  std::thread disparity(disparityCalcSGBM,std::ref(s),std::ref(disparitySGBM));

  // load disparity parameters to get intial values
  if(!loadDisparityParameters("./configs/sgbm.yml"))
    return 0;

  // create subimage container to save created subdivisions
  std::vector<std::vector<cv::Mat>> subimages;
  subimages.reserve(9);

  // init obstacle detection
  obstacleDetection obst;

  running = true;
  int frame = 0;
  while(running)
  {
    stereo.getRectifiedImagepair(s);
    // std::cout << frame << std::endl;
    cv::imshow("Left", s.mLeft);
    cv::imshow("Right", s.mRight);

    // notify the thread to start 
    cond_var.notify_one();

    if(newDisparityMap)
    {
      // cut the disparity map in order to ignore the camera shift
      // cv::Rect dMapROI = cv::Rect(cv::Point(numDisp/3,0),
                                  // cv::Point(dMapRaw.cols,dMapRaw.rows));
      // dMapRaw = dMapRaw(dMapROI);

      // clear current subimages and refill the container with new ones
      subimages.clear();
      // subdivideImages(dMapRaw, subimages, binning);
      // obst.buildMeanDistanceMap(Q_32F, subimages, binning);

      // display stuff
      cv::normalize(dMapRaw,dMapNorm,0,255,cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(dMapNorm,dMapNorm,CV_GRAY2BGR);
      // View::drawObstacleGrid(dMapNorm, binning);
      // View::drawSubimageGrid(dMapNorm, binning);
      cv::imshow("SGBM",dMapNorm);
      newDisparityMap = false;
    }

    key = cv::waitKey(5);

    // keypress stuff
    if(key > 0)
    {
      switch(key)
      {
        case 'q':
          LOG(INFO) << tag << "Exit requested" <<std::endl;
          delete left;
          left  = nullptr;
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
          std::cout<< left->getFramerate() << " " << right-> getFramerate() <<std::endl;
          break;
        case 'r':
          loadDisparityParameters("./configs/sgbm.yml");
          break;
        case '0':
          std::cout << obst.getDistanceMapMean()[41] << std::endl;
          break;
        case 'E':
          exposure += 1000;
          left->setExposure(exposure);
          right->setExposure(exposure);
          break;
        case 'e':
          exposure -= 1000;
          left->setExposure(exposure);
          right->setExposure(exposure);
          break;
        case 'G':
          gain += 0.5;
          left->setGain(gain);          
          right->setGain(gain);
          break;
        case 'g':
          gain -= 0.5;
          left->setGain(gain);          
          right->setGain(gain);
          break;
        case 'h':
          if(!hdr){
            left->enableHDR(Camera::hdr::ENABLE_HDR);
            right->enableHDR(Camera::hdr::ENABLE_HDR);
            hdr = true;
          } else {
            left->enableHDR(Camera::hdr::DISABLE_HDR);
            right->enableHDR(Camera::hdr::DISABLE_HDR);
            hdr = false;
          }
        case 'n':
          {
            cv::FileStorage g("newStuff.yml", cv::FileStorage::WRITE);
            g << "Q" << Q_32F;
            g << "K_L" << stereo.getNewKMats()[0];
            g << "K_R" << stereo.getNewKMats()[1];
          }
        default:
          std::cout << "Key pressed has no action" << std::endl;
          break;
      }
    }
    ++frame;
  }

  disparity.join();
  return 0;
}
