#include <iostream>
#include <string>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"

#include <thread>
#include <mutex>
#include <condition_variable>

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started.";
  mvIMPACT::acquire::DeviceManager devMgr;

  Camera *left;
  Camera *right;

  if(!Utility::initCameras(devMgr,left,right))
  {
    return 0;
  }

  Stereosystem stereo(left,right);

  if(!stereo.loadIntrinsic("parameter/intrinsic.yml"))
  {
    std::cout << "foo" << std::endl;
    return 0;
  }
  if(!stereo.loadExtrinisic("parameter/extrinsic.yml"))
  {
    std::cout << "foo" << std::endl;
    return 0;
  }

  Stereopair s;

  left->setExposure(24000);
  right->setExposure(24000);

  char key = 0;
  int binning = 0;
  cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  bool running = true;
  while(running)
  {

    if(!stereo.getUndistortedImagepair(s))
    {
      std::cout << "foo" << std::endl;
      break;
    }
    cv::imshow("Left", s.mLeft);
    cv::imshow("Right", s.mRight);

    key = cv::waitKey(5);

    if(key > 0)
    {
      switch(key)
      {
        case 'q':
          running = false;
          LOG(INFO) << tag << "Exit requested" <<std::endl;
          delete left;
          left = NULL;

          delete right;
          right = NULL;
          break;
        case 'b':
          if (binning == 0)
            binning = 1;
          else
            binning =0;

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
  }


  return 0;
}
