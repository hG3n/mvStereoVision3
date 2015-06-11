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

cv::StereoSGBM disparitySGBM;

int numDispSGBM = 64;
int windSizeSGBM = 9;
int exposure = 4800;

Camera *left;
Camera *right;

cv::Mat R, R_32F;
cv::Mat Q, Q_32F;

std::string leftText;
std::string qText;
std::string worldText;
Stereopair s;

//callback function for opencv slider to change the number of disparity parameter of SGBM
void changeNumDispSGBM(int, void*)
{
  //numDisp has to be dividable by 16, constraint from opencv
  numDispSGBM+=numDispSGBM%16;

  //numDisp is not allowed to be smaller than 16
  if(numDispSGBM < 16)
  {
    numDispSGBM = 16;
    cv::setTrackbarPos("Num Disp", "SGBM", numDispSGBM);
  }

  cv::setTrackbarPos("Num Disp", "SGBM", numDispSGBM);
  //rewrite the disparitySGBM object
  disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
}

//callback function for opencv slider to change the window size paramter of SGBM
void changeWindSizeSGBM(int, void*)
{
  //windsize has to be odd...
  if(windSizeSGBM%2 == 0)
    windSizeSGBM+=1;

  //...and bigger than 5
  if(windSizeSGBM < 5)
  {
    windSizeSGBM = 5;
    cv::setTrackbarPos("Wind Size", "SGBM", windSizeSGBM);
  }
  cv::setTrackbarPos("Wind Size", "SGBM", windSizeSGBM);
  //rewrite the disparitySGBM object
  disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
}

//callback function for opencv slider to change the exposure of the cameras
void setExposure(int, void*)
{
    left->setExposure(exposure);
    right->setExposure(exposure);
}

//mouse callback function for opencv
void mouseClick(int event, int x, int y,int flags, void* userdata)
{

  //on mouseclick calculate distance of chosen pixel and put it on cout
  if  ( event == CV_EVENT_LBUTTONDOWN )
     {
        cv::Mat_<float>  coordinateQ(1,4);
        if(Utility::calcCoordinate(coordinateQ ,Q_32F ,dispMapSGBM,x,y))
        {
            std::cout<<"Coordinate Q: " << coordinateQ <<std::endl;
            std::cout<<"distance Q: " << (coordinateQ(2)/1000.0) <<std::endl;
        }
        //coordinate is false if the value is negative (e.g.-1) so no information is available at this pint
        else
        {
            std::cout <<"invalid disparity\n";
        }

     }
     //on mouseover calculate distance of chosen pixel and put it onto the image
     else if ( event == CV_EVENT_MOUSEMOVE )
     {
        cv::Mat_<float>  coordinateQ(1,4);
        if(Utility::calcCoordinate(coordinateQ ,Q_32F ,dispMapSGBM,x,y))
        {
            qText = "distance Q: " + std::to_string(coordinateQ(2)/1000.0);
        }
        else
        {
            std::cout <<"invalid disparity\n";
        }
     }
}


//create the windows and slider and register the callback functions
void initWindows()
{
    cv::namedWindow("SGBM" ,CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Left" ,CV_WINDOW_AUTOSIZE);

    cv::createTrackbar("Num Disp", "SGBM", &numDispSGBM, 320, changeNumDispSGBM);
    cv::createTrackbar("Wind Size", "SGBM", &windSizeSGBM, 51, changeWindSizeSGBM);
    cv::createTrackbar("Exposure", "SGBM", &exposure, 48000, setExposure);

    cv::setMouseCallback("SGBM", mouseClick, NULL);
    cv::setMouseCallback("Left", mouseClick, NULL);
}



int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started.";
  mvIMPACT::acquire::DeviceManager devMgr;

  initWindows();

  //init the cameras
  if(!Utility::initCameras(devMgr,left,right))
  {
    return 0;
  }

  //init the stereo system AFTER init the cameras!
  Stereosystem stereo(left,right);


  //collect some configuration parameter written in /configs/default.yml to set "something"
  std::vector<std::string> nodes;
  //"something" is here the extrinsic and intrinsic parameters...
  nodes.push_back("inputParameter");
  //... and the location where disparity maps should be stored
  nodes.push_back("capturedDisparities");

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

  std::string outputDirectory;
  fs["capturedDisparities"] >> outputDirectory;


  //load the camera matrices, dist coeffs, R, T, ....
  if(!stereo.loadIntrinsic(inputParameter+"/intrinsic.yml"))
    return 0;
  if(!stereo.loadExtrinisic(inputParameter +"/extrinsic.yml"))
    return 0;


  int imageNumber = 0;
  //check if the given directory exists
  if(Utility::directoryExist(outputDirectory))
  {
    std::vector<std::string> tmp;
    //check if the output directory is empty
    Utility::getFiles(outputDirectory,tmp);


    if(tmp.size() != 0)
    {
      //if the directory is not empty, clean all files=
      std::cout << "Output directory not empty, clean files? [y/n] " <<std::endl;
      char key = getchar();
      if(key == 'y')
      {
        std::system(std::string("rm " + outputDirectory + "/* -f ").c_str());
      }
      //start image counter at the last image number in the directory
      else if(key == 'n')
      {
        imageNumber = tmp.size();
        LOG(INFO) << tag << "Start with image number " << imageNumber << std::endl;
      }
    }
  }

  //if the output directory doesn't exist->create it
  if(Utility::createDirectory(outputDirectory))
  {
    LOG(INFO) << tag << "Successfully created directory for captured disparity maps." << std::endl;
  }
  else
  {
    LOG(ERROR) << tag << "Unable to create directory for captured disparity maps." <<std::endl;
    return 0;
  }


  //the SGBM object
  disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);

  stereo.getRectifiedImagepair(s);

  R = stereo.getRotationMatrix();
  Q = stereo.getQMatrix();

  //convert the matrices because otherwise opencv gives you crap
  R.convertTo(R_32F,CV_32F);
  Q.convertTo(Q_32F,CV_32F);

  int key = 0;
  int binning =0;

  cv::Mat normalizedSGBM;
  cv::Mat leftColor;
  while(running)
  {
    stereo.getRectifiedImagepair(s);

    //convert image to color, just for nice text color on the image
    cvtColor(s.mLeft, leftColor, CV_GRAY2RGB);
    //put the string with the distance on the image
    cv::putText(leftColor,qText.c_str(), cv::Point(10,80),CV_FONT_HERSHEY_PLAIN,2,CV_RGB(255,128,0));

    cv::imshow("Left", leftColor);

    //calculate the disparity
    Disparity::sgbm(s, dispMapSGBM, disparitySGBM);
    //normalize it for viewing purposes
    cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
    cv::imshow("SGBM",normalizedSGBM);

    key = cv::waitKey(10);
    //quit program
    if(char(key) == 'q')
    {
      running = false;
    }
    //binning mode
    else if(char(key) == 'b')
    {
      if (binning == 0)
      {
        binning = 1;
      }
      else
      {
        binning =0;
      }

      left->setBinning(binning);
      right->setBinning(binning);
      stereo.resetRectification();
    }
    //capture images
    else if (char(key) == 'c')
    {
      std::string prefix = "";
      if(imageNumber < 10)
      {
        prefix +="000";
      }
      else if ((imageNumber >=10) && (imageNumber <100))
      {
        prefix += "00";
      }
      else if(imageNumber >= 100)
      {
        prefix +="0";
      }
      cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
      cv::imwrite(std::string(outputDirectory+"/sgbm_raw_"+prefix+std::to_string(imageNumber)+".jpg"),dispMapSGBM);
      cv::imwrite(std::string(outputDirectory+"/sgbm_norm_"+prefix+std::to_string(imageNumber)+".jpg"),normalizedSGBM);

      ++imageNumber;
    }
    //get frame rate
    else if(char(key) == 'f')
      std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;
  }

return 0;
}
