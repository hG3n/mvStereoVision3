#include <iostream>
#include <string>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"

#include <thread>
#include <mutex>
#include <condition_variable>

//init the logger
INITIALIZE_EASYLOGGINGPP


int main(int argc, char* argv[])
{
  //tag for LOG-information
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started.";

  //create a device manager for the matrix vision cameras
  mvIMPACT::acquire::DeviceManager devMgr;

  //create cameras
  Camera *left;
  Camera *right;

  //all the driver stuff from matrix vision fo rthe cameras is done here
  if(!Utility::initCameras(devMgr,left,right))
    return 0;


  //init the stereo system AFTER init the cameras!
  Stereosystem stereo(left,right);

  //collect some configuration parameter written in /configs/default.yml to set "something"
  std::vector<std::string> nodes;
  //"something" is here the extrinsic and intrinsic parameters
  nodes.push_back("inputParameter");


  std::string config = "./configs/default.yml";

  cv::FileStorage fs;

  //check if the config has all the collected configuration parameters
  if(!Utility::checkConfig(config,nodes,fs))
  {
    return 0;
  }

  //put the collected paramters to some variable
  std::string inputParameter;
  fs["inputParameter"] >> inputParameter;

  //load the camera matrices, dist coeffs, R, T, ....
  if(!stereo.loadIntrinsic(inputParameter+"/intrinsic.yml"))
    return 0;
  if(!stereo.loadExtrinisic(inputParameter +"/extrinsic.yml"))
    return 0;

  //set the exposure time for left and right camera
  left->setExposure(24000);
  right->setExposure(24000);


  char key = 0;
  bool running = true;
  unsigned int binning = 0;

  //create windows
  cv::namedWindow("Left Distorted" ,1);
  cv::namedWindow("Right Distorted" ,1);

  cv::namedWindow("Left Undistorted" ,1);
  cv::namedWindow("Right Undistorted" ,1);

  cv::namedWindow("Left Rectified" ,1);
  cv::namedWindow("Right Rectified" ,1);

  //Stereopair is a struct holding a left and right image
  Stereopair distorted;
  Stereopair undistorted;
  Stereopair rectified;


  //endless loop until 'q' is pressed
  while(running)
  {

    //get the different types of images from the stereo system...
    stereo.getImagepair(distorted);
    stereo.getUndistortedImagepair(undistorted);
    stereo.getRectifiedImagepair(rectified);

    //... and show them all (with the stereopair.mLeft/mRight you have acces to the images and can do whatever you want with them)
    cv::imshow("Left Distorted", distorted.mLeft);
    cv::imshow("Right Distorted", distorted.mRight);

    cv::imshow("Left Undistorted", undistorted.mLeft);
    cv::imshow("Right Undistorted", undistorted.mRight);

    cv::imshow("Left Rectified", rectified.mLeft);
    cv::imshow("Right Rectified", rectified.mRight);

    key = cv::waitKey(10);

    //quit program
    if(key == 'q')
    {
      running = false;
      break;
    }
    //switch binning mode
    else if(key == 'b')
    {
    if (binning == 0)
      binning = 1;
    else
      binning =0;
      //after binning the rectificaction has to reseted to have the correct parameters
      left->setBinning(binning);
      right->setBinning(binning);
      stereo.resetRectification();
    }
    //show FPS of the cameras
    else if(key == 'f')
    {
     std::cout << "FPS: Right "<< left->getFramerate() << " " << "Left " << right->getFramerate() << std::endl;
    }
    //capture the images anbd write them to disk
    else if (key == 'c')
    {
      cv::imwrite("Left Distorted.jpg",distorted.mLeft);
      cv::imwrite("Right Distorted.jpg",distorted.mRight);

      cv::imwrite("Left Undistorted.jpg",undistorted.mLeft);
      cv::imwrite("Right Undistorted.jpg",undistorted.mRight);

      cv::imwrite("Left Rectified.jpg",rectified.mLeft);
      cv::imwrite("Right Rectified.jpg",rectified.mRight);
    }
  }
  return 0;
}
