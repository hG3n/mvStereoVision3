#include <iostream>
#include <string>
#include <chrono>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"
#include "view.h"

#include "ObstacleDetection.h"
#include "MeanDisparityDetection.h"
#include "SamplepointDetection.h"

#include "Samplepoint.h"

#include <thread>
#include <mutex>
#include <condition_variable>

INITIALIZE_EASYLOGGINGPP
std::string tag = " Main ";
bool running = true;

// threading stuff
std::mutex disparityLock;
std::condition_variable cond_var;

cv::Ptr<cv::StereoSGBM> disparitySGBM;

// create empty dto for storing sgbm parameters
Disparity::sgbmParameters sgbmParameters;
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

int exposure = 10000;
float gain = 4.0;
bool hdr = false;

bool detectionIsInit = true;

void disparityCalcSGBM(Stereopair const& s, cv::Ptr<cv::StereoSGBM> disparity) {
  while(running)
  {
    std::unique_lock<std::mutex> ul(disparityLock);
    cond_var.wait(ul);
    Disparity::sgbm(s, dMapRaw, disparity);
    dMapRaw.convertTo(disparityMap_32FC1,CV_32FC1);
    newDisparityMap = true;
  }
}

void mouseClick(int event, int x, int y,int flags, void* userdata) {
  if  ( event == CV_EVENT_LBUTTONDOWN )
  {
    double d = static_cast<float>(dMapRaw.at<short>(y,x));
    float distance = Utility::calcDistance(Q_32F, d, 1);
    // std::cout << "disparityValue: " << d << "  distance: " << distance << std::endl;

    // cv::Mat_<float> vec = Utility::calcCoordinate(Q_32F, d, x ,y);
    // std::cout << vec << std::endl;
    // std::cout << "coordinate: " << vec << std::endl;

    // define center point 
    cv::Mat_<float> center(1,4);
    center(0) = 0.0f;
    center(1) = 0.0f;
    center(2) = 1.0f;
    center(3) = 0.0f;

    // std::cout << "angle " << Utility::calcAngle(center, vec) << std::endl;

    cv::Mat_<float> back(4,1);
    back(0) = 0.0f;
    back(1) = 0.0f;
    back(2) = 3000.0f;
    back(3) = 0.0f;

    std::cout << back << std::endl;
    std::cout << Q_32F << std::endl;

    cv::Mat_<float> newMat = Q_32F / back;


  }
}

void subdivideImages(cv::Mat const& dMap, std::vector<std::vector<cv::Mat>> &subimages, int binning) {
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

void createDMapROIS(cv::Mat const& reference, cv::Rect & roi_u, cv::Rect& roi_b, int binning, bool reload = false) {
  /* TODO:
    PUT EVERYTHING TO DISPARITY NAMESPACE
  */

  // the best value to work with in order to include everything and dont exclude important work
  int pixelShift = sgbmParameters.numDisp / 2;
  if(pixelShift % 2 == 1) {
    pixelShift = pixelShift + 1;
    if((reference.cols - pixelShift) % 8 != 0) {
      pixelShift = pixelShift + (reference.cols - pixelShift % 8);
    }
  }

  std::cout << reference.size() << std::endl;

  if(!reload) {
    roi_u = cv::Rect(cv::Point((pixelShift),0),cv::Point(reference.cols,reference.rows));
    roi_b = cv::Rect(cv::Point((pixelShift)/2,0),cv::Point(reference.cols/2,reference.rows/2));
  } else {
    if(binning == 0){
      roi_u = cv::Rect(cv::Point(pixelShift,0), cv::Point(reference.cols,reference.rows));
    } else {
      roi_b = cv::Rect(cv::Point(pixelShift,0), cv::Point(reference.cols,reference.rows));
    }
  }

  std::cout << "roi b" << roi_b << std::endl;
  std::cout << "roi u" << roi_u << std::endl;
}

void createSamplepointArray(std::vector<Samplepoint>& toFill, cv::Mat const& reference) {
  int distanceX = reference.cols/8;
  int distanceY = reference.rows/8;

  std::cout << reference.cols/distanceX << std::endl;
  std::cout << reference.cols/distanceY << std::endl;

  for (int c = 1; c < distanceX; ++c)
  {
    for (int r = 1; r < distanceY; ++r)
    { 
      std::cout << "c: " << c << " r: " << r << std::endl;
      toFill.push_back(Samplepoint(cv::Point(c*(reference.cols/distanceX), r*(reference.rows/distanceY)), 1));
    }
  }
}

int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started." << std::endl;
  mvIMPACT::acquire::DeviceManager devMgr;

  // create both cameras
  // auto left = std::make_shared<Camera>;
  // auto right = std::make_shared<Camera>;
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

  // create disparity object and connected thread
  disparitySGBM = cv::StereoSGBM::create(sgbmParameters.minDisp,
                                         sgbmParameters.numDisp,
                                         sgbmParameters.blockSize,
                                         8*sgbmParameters.blockSize*sgbmParameters.blockSize,
                                         32*sgbmParameters.blockSize*sgbmParameters.blockSize);
  std::thread disparity(disparityCalcSGBM,std::ref(s),std::ref(disparitySGBM));

  // load disparity parameters to get intial values
  // if(!loadDisparityParameters("./configs/sgbm.yml"))
  //   return 0;

  if(!Disparity::loadSGBMParameters("./configs/sgbm.yml", disparitySGBM, sgbmParameters))
    return 0;

  // pre create the rois for binned and unbinned mode
  cv::Rect dMapROI_b, dMapROI_u;
  createDMapROIS(dMapWork, dMapROI_u, dMapROI_b, 0);

  // create subimage container to save created subdivisions
  std::vector<std::vector<cv::Mat>> subimages;
  subimages.reserve(9);

  ObstacleDetection *o;
  MeanDisparityDetection m;
  SamplepointDetection sd;
  
  m.init(Q_32F);
  // o = &m;
  sd.init(dMapWork, Q_32F);
  o = &sd;

  running = true;
  int frame = 0;
  while(running)
  {
    stereo.getRectifiedImagepair(s);
    cv::imshow("Left", s.mLeft);
    cv::imshow("Right", s.mRight);

    // notify the thread to start 
    cond_var.notify_one();

    if(newDisparityMap)
    {
      // cut the disparity map in order to ignore the camera shift
      // camera shift is roughly calculated by a third of numDisparity
      // added up with the used blocksize
      if(reload) {
        std::cout << "dMapRaw size: " << dMapRaw.size() << std::endl;
        createDMapROIS(dMapRaw, dMapROI_u, dMapROI_b, binning, reload);
        reload = false;
      }

      if(binning == 0) {
        dMapRaw.convertTo(dMapWork, CV_32F);
        dMapWork = dMapRaw(dMapROI_u);

        if(detectionIsInit){
          Q = stereo.getQMatrix();
          Q.convertTo(Q_32F,CV_32F);
          sd.init(dMapWork, Q_32F);
          o = &sd;
          detectionIsInit = false;
        }

      } else if (binning == 1) {
        dMapRaw.convertTo(dMapWork, CV_32F);
        dMapWork = dMapRaw(dMapROI_b);
   
        if(detectionIsInit) {
          Q = stereo.getQMatrix();
          Q.convertTo(Q_32F,CV_32F);
          sd.init(dMapWork, Q_32F);
          o = &sd;
          detectionIsInit = false;
        }
      }

      o->build(dMapWork, binning, MeanDisparityDetection::MODE::MEAN_DISTANCE);
      // o->detectObstacles();

      // display stuff
      cv::normalize(dMapWork,dMapNorm,0,255,cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(dMapNorm,dMapNorm,CV_GRAY2BGR);

      if (view % 2 == 0) {
        View::drawSubimageGrid(dMapNorm, binning);
        View::drawObstacleGrid(dMapNorm, binning);
      }

      cv::imshow("SGBM",dMapNorm);
      newDisparityMap = false;
    }

    key = cv::waitKey(5);

    // keypress stuff
    if(key > 0)
    {
      switch(key) {
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
        {
          if (binning == 0)
            binning = 1;
          else
            binning = 0;
          left->setBinning(binning);
          right->setBinning(binning);
          stereo.resetRectification();
          detectionIsInit = true;
          reload = true;
          // reload parameters in order to recompute pixelshift
          Disparity::loadSGBMParameters("./configs/sgbm.yml", disparitySGBM, sgbmParameters);
          break;
        }
        case 'f':
          std::cout<< left->getFramerate() << " " << right-> getFramerate() <<std::endl;
          break;
        case 'r':
          Disparity::loadSGBMParameters("./configs/sgbm.yml", disparitySGBM, sgbmParameters);
          detectionIsInit = true;
          reload = true;
          break;
        case '0':
          o->detectObstacles();
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
            break;
          } else {
            left->enableHDR(Camera::hdr::DISABLE_HDR);
            right->enableHDR(Camera::hdr::DISABLE_HDR);
            hdr = false;
            break;
          }
          break;
        case 'd':
          o->detectObstacles();
          break;
        case 'n':
        {
          cv::FileStorage g("afterCalibrationParameters.yml", cv::FileStorage::WRITE);
          g << "Q" << Q_32F;
          g << "K_L" << stereo.getNewKMats()[0];
          g << "K_R" << stereo.getNewKMats()[1];
          g.release();
          break;
        }
        case 'v':
            ++view;
            break;
        default:
          std::cout << "Key pressed has no action" << std::endl;
          break;
      }
    }
    ++frame;
  }

  // release matrices 
  dMapRaw.release();
  dMapNorm.release();
  dMapWork.release();

  // end the disparity thread
  disparity.join();
  return 0;
}
