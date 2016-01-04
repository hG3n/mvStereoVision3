#include <iostream>
#include <string>
#include <chrono>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"
#include "view.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#include <ctime>

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

bool capture = false;

std::string const disparity_path = "images/disparity";

void disparityCalcSGBM(Stereopair const& s, cv::Ptr<cv::StereoSGBM> disparity)
{
  while(running)
  {
    std::unique_lock<std::mutex> ul(disparityLock);
    cond_var.wait(ul);
    Disparity::sgbm(s, dMapRaw, disparity);
    dMapRaw.convertTo(disparityMap_32FC1,CV_32FC1);
    newDisparityMap = true;
  }
}

void createDMapROIS(cv::Mat const& reference, cv::Rect & roi_u, cv::Rect& roi_b, int binning, bool reload = false)
{
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

int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started." << std::endl;
  mvIMPACT::acquire::DeviceManager devMgr;

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
  Stereopair s;

  char key = 0;
  int binning = 0;

  // get one single imagepair in order to receive the Q Matrix
  // which is created during the rectification
  stereo.getRectifiedImagepair(s);
  Q = stereo.getQMatrix();
  Q.convertTo(Q_32F,CV_32FC1);

  // init OpenCV GUI objects
  cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("SGBM" ,1);

  // create disparity object and connected thread
  disparitySGBM = cv::StereoSGBM::create(sgbmParameters.minDisp,
                                         sgbmParameters.numDisp,
                                         sgbmParameters.blockSize,
                                         8*sgbmParameters.blockSize*sgbmParameters.blockSize,
                                         32*sgbmParameters.blockSize*sgbmParameters.blockSize);
  std::thread disparity(disparityCalcSGBM,std::ref(s),std::ref(disparitySGBM));

  // load parameters for sgbm
  if(!Disparity::loadSGBMParameters("./configs/sgbm.yml", disparitySGBM, sgbmParameters))
    return 0;

  // pre create the rois for binned and unbinned mode
  cv::Rect dMapROI_b, dMapROI_u;
  createDMapROIS(dMapWork, dMapROI_u, dMapROI_b, 0);

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

      } else if (binning == 1) {
        dMapRaw.convertTo(dMapWork, CV_32F);
        dMapWork = dMapRaw(dMapROI_b);
      }

      // capturing stuff
      if(capture)
      {
        std::string prefix = "";
        if(frame < 10)
        {
          prefix +="000";
        }
        else if ((frame >=10) && (frame <100))
        {
          prefix += "00";
        }
        else if(frame >= 100)
        {
          prefix +="0";
        }
        cv::imwrite(std::string(disparity_path+"/left_"+prefix+std::to_string(frame)+".jpg"),dMapRaw);
        ++frame;
      }

      // display stuff
      cv::normalize(dMapWork,dMapNorm,0,255,cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(dMapNorm,dMapNorm,CV_GRAY2BGR);

      if (view % 3 == 0) {
        View::drawSubimageGrid(dMapNorm, binning);
        View::drawObstacleGrid(dMapNorm, binning);
      }

      cv::imshow("SGBM",dMapNorm);
      newDisparityMap = false;
    }

    key = cv::waitKey(2);

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
          reload = true;
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
        case 'c':
          if(!capture)
            capture = true;
          else
            capture = false;
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
        default:
          std::cout << "Key pressed has no action" << std::endl;
          break;
      }
    }
  }

  // release matrices 
  dMapRaw.release();
  dMapNorm.release();
  dMapWork.release();

  // end the disparity thread
  disparity.join();
  return 0;
}

/*
		

		if(key == 'q')
		{
			running = false;
			break;
		}
		else if(key == 'b')
		{
		if (binning == 0)
			binning = 1;
		else
			binning =0;

		 left->setBinning(binning);
		 right->setBinning(binning);
		 stereo.resetRectification();
		}
		else if(key == 'f')
		{
		 std::cout << "FPS: Right "<< left->getFramerate() << " " << "Left " << right->getFramerate() << std::endl;
		}
		else if (key == 'c')
		{
			capture = !capture;
			if(capture)
			{
				LOG(INFO) << tag << "Capture enabled" << std::endl;
			}
			else
			{
				LOG(INFO) << tag << "Capture disabled" << std::endl;
			}
		}
	}
	return 0;
}*/
