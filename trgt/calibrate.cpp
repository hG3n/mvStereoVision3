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

// working mode
int mode = 0;

// camera variables
int exposure = 10000;
float gain = 4.0;
bool hdr = false;

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started.";
  mvIMPACT::acquire::DeviceManager devMgr;

  // intialize cameras, stereo system and stereopair
  Camera *left;
  Camera *right;

  if(!Utility::initCameras(devMgr,left,right)) {
    return 0;
  }

  Stereosystem stereo(left,right);
  Stereopair s;

  // config settings
  std::vector<std::string> nodes;
  nodes.push_back("outputImages");

  std::string config = "./configs/default.yml";
  cv::FileStorage fs;

  if(!Utility::checkConfig(config,nodes,fs)) {
    return 0;
  }

  std::string dirPath;
  fs["outputImages"] >> dirPath;

  std::string parameterOutput;
  fs["outputParameter"] >> parameterOutput;

  std::string pathLeft = dirPath+"/left";
  std::string pathRight = dirPath+"/right";

  // init filename containers
  std::vector<std::string> filenamesLeft;
  std::vector<std::string> filenamesRight;

  std::vector<cv::Mat> imagesLeft, imagesRight;

  int imageNumber = 0;

  // check if the directories exist
  if(Utility::directoryExist(pathLeft) || Utility::directoryExist(pathRight)) {
    std::vector<std::string> tmp1, tmp2;
    Utility::getFiles(pathLeft,tmp1);
    Utility::getFiles(pathRight,tmp2);

    // if current directories do exist ask for permission to delete files or leave them in there
    if(tmp1.size() != 0 || tmp2.size() != 0) {
      std::cout << "Output directory not empty, clean files? [y/n] " <<std::endl;
      char key = getchar();
      if(key == 'y') {
        std::system(std::string("rm " + pathLeft + "/* -f ").c_str());
        std::system(std::string("rm " + pathRight + "/* -f ").c_str());
      } else if(key == 'n') {
        imageNumber = tmp1.size();
        LOG(INFO) << tag << "Start with image number " << imageNumber << std::endl;
      }
    }
  }

  if(Utility::createDirectory(pathLeft) && Utility::createDirectory(pathRight)) {
      LOG(INFO) << tag << "Successfully created directories for captured images." << std::endl;
  } else {
    LOG(ERROR) << tag << "Unable to create directories for captured images." <<std::endl;
    return 0;
  }

  // define initial camera settings
  left->setExposure(12000);
  right->setExposure(12000);

  left->setPixelClock(1);
  right->setPixelClock(1);

  char key = 0;
  int binning = 0;
  cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  bool running = true;
  std::string filename = "";
  while(running)
  {
    /******************
     * CAPTURING MODE *
     ******************/
    if(mode == 0) {
      // get image pair
      if(!stereo.getImagepair(s)) {
        LOG(ERROR) << tag << "Unable to get imagepair" << std::endl;
        break;
      }

      // show images
      cv::imshow("Left", s.mLeft);
      cv::imshow("Right", s.mRight);

      // mode specific key presses
      if(key > 0 ) {
        switch(key) {
          case 'b':
            // switch binning mode
            if (binning == 0)
              binning = 1;
            else
              binning =0;

            left->setBinning(binning);
            right->setBinning(binning);
            stereo.resetRectification();
            break;
          case 'c':
            // capture image
            filename = "";
            if(imageNumber < 10) {
              filename+= "00" + std::to_string(imageNumber);
            }
            if(imageNumber >= 10 && imageNumber < 100) {
              filename+="0" + std::to_string(imageNumber);
            }

            cv::imwrite(pathLeft+"/left_"+filename+".jpg",s.mLeft);
            cv::imwrite(pathRight+"/right_"+filename+".jpg",s.mRight);
            LOG(INFO) << tag << "Wrote left image to " << std::string(pathLeft+"/left_"+filename+".jpg") <<std::endl;
            LOG(INFO) << tag << "Wrote right image to " << std::string(pathRight+"/right_"+filename+".jpg") <<std::endl;
            ++imageNumber;
            break;
          case 'f':
            std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;
            break;
        }
      }

    /********************
     * CALIBRATION MODE *
     ********************/
    } else if (mode == 1) {

      LOG(INFO) << tag << "Starting calibration process" << std::endl;
      // locd previous captured images
      if(filenamesLeft.size() == filenamesRight.size()) {
        for(unsigned int i = 0; i < filenamesLeft.size(); ++i){
          imagesLeft.push_back(cv::imread(std::string(dirPath+"/left/"+filenamesLeft[i])));
          imagesRight.push_back(cv::imread(std::string(dirPath+"/right/"+filenamesRight[i])));
        }
      }

      // calibrate images
      double stereoRMS = stereo.calibrate(imagesLeft, imagesRight,20, cv::Size(11,8));
      std::cout << "RMS after stereorectification: " <<  stereoRMS << std::endl;

      std::cout << "press 's' to save the new parameters." << std::endl;
      char key = getchar();
      if(key == 's')
      {
        stereo.saveIntrinsic(parameterOutput + "/intrinsic.yml");
        stereo.saveExtrinsic(parameterOutput + "/extrinsic.yml");
      }

      ++mode;

    /**************
     * FINAL MODE *
     **************/
    } else if (mode == 2) {
      printf("final mode");
    }

    key = cv::waitKey(5);
    if(key > 0)
    {
      switch(key)
      {
        case 'q':
          // quit program
          running = false;
          LOG(INFO) << tag << "Exit requested" <<std::endl;
          delete left;
          left = NULL;
          delete right;
          right = NULL;
          break;
        case 'n':
          if(mode == 0) {
            Utility::getFiles(dirPath+"/left", filenamesLeft);
            Utility::getFiles(dirPath+"/right", filenamesRight);
          }
          // skip to next input mode
          ++mode;
        default:
          std::cout << "Key pressed has no action" <<std::endl;
          break;
      }
    }
  }


  return 0;
}
