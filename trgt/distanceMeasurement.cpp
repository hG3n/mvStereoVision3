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
cv::Mat dispMapSGBM, dispMapBM, dispMapNCC;

cv::StereoSGBM disparitySGBM;

int numDispSGBM = 64;
int windSizeSGBM = 9;
int exposure = 4800;

Camera *left;
Camera *right;


double baseline ;
double fx ;
double fy ;
double cx ;
double cy ;
double scale = 1;

cv::Mat R, R_32F;
cv::Mat Q, Q_32F;

std::string leftText;
std::string qText;
std::string worldText;
Stereopair s;

void changeNumDispSGBM(int, void*)
{
  numDispSGBM+=numDispSGBM%16;

  if(numDispSGBM < 16)
  {
    numDispSGBM = 16;
    cv::setTrackbarPos("Num Disp", "SGBM", numDispSGBM);
  }

  cv::setTrackbarPos("Num Disp", "SGBM", numDispSGBM);
  disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
}

void changeWindSizeSGBM(int, void*)
{
  if(windSizeSGBM%2 == 0)
    windSizeSGBM+=1;

  if(windSizeSGBM < 5)
  {
    windSizeSGBM = 5;
    cv::setTrackbarPos("Wind Size", "SGBM", windSizeSGBM);
  }
  cv::setTrackbarPos("Wind Size", "SGBM", windSizeSGBM);
  disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
}

void setExposure(int, void*)
{
    left->setExposure(exposure);
    right->setExposure(exposure);
}

void mouseClick(int event, int x, int y,int flags, void* userdata)
{

  if  ( event == CV_EVENT_LBUTTONDOWN )
     {
        cv::Mat_<float>  coordinateQ(1,4);
        if(Utility::calcCoordinate(coordinateQ ,Q_32F ,dispMapSGBM,x,y))
        {
            std::cout<<"Coordinate Q: " << coordinateQ <<std::endl;
            std::cout<<"distance Q: " << (coordinateQ(2)/1000.0) <<std::endl;
        }
        else
        {
            std::cout <<"invalid disparity\n";
        }

     }
     // else if  ( event == CV_EVENT_RBUTTONDOWN )
     // {
     //      std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     // }
     // else if  ( event == CV_EVENT_MBUTTONDOWN )
     // {
     //      std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     // }
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


void initWindows()
{
    cv::namedWindow("SGBM" ,CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Left" ,CV_WINDOW_AUTOSIZE);

    //cv::namedWindow("BM" ,1);

    cv::createTrackbar("Num Disp", "SGBM", &numDispSGBM, 320, changeNumDispSGBM);
    cv::createTrackbar("Wind Size", "SGBM", &windSizeSGBM, 51, changeWindSizeSGBM);
    cv::createTrackbar("Exposure", "SGBM", &exposure, 48000, setExposure);


  // cv::createTrackbar("Num Disp", "BM", &numDispBM, 320, changeNumDispBM);
  // cv::createTrackbar("Wind Size", "BM", &windSizeBM, 51, changeWindSizeBM);


    cv::setMouseCallback("SGBM", mouseClick, NULL);
    cv::setMouseCallback("Left", mouseClick, NULL);
}


int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started.";
  mvIMPACT::acquire::DeviceManager devMgr;

    initWindows();

  if(!Utility::initCameras(devMgr,left,right))
  {
    return 0;
  }

  left->setExposure(4800);
  right->setExposure(4800);

  std::vector<std::string> nodes;
  nodes.push_back("inputParameter");
  nodes.push_back("capturedDisparities");

  std::string config = "./configs/default.yml";

  cv::FileStorage fs;
  left->setExposure(20000);
  right->setExposure(20000);

  if(!Utility::checkConfig(config,nodes,fs))
  {
  return 0;
  }

  std::string inputParameter;
  fs["inputParameter"] >> inputParameter;

  std::string outputDirectory;
  fs["capturedDisparities"] >> outputDirectory;

  int imageNumber = 0;

  if(Utility::directoryExist(outputDirectory))
  {
    std::vector<std::string> tmp1;
    Utility::getFiles(outputDirectory,tmp1);

    if(tmp1.size() != 0)
    {
      std::cout << "Output directory not empty, clean files? [y/n] " <<std::endl;
      char key = getchar();
      if(key == 'y')
      {
        std::system(std::string("rm " + outputDirectory + "/* -f ").c_str());
      }
      else if(key == 'n')
      {
        imageNumber = tmp1.size();
        LOG(INFO) << tag << "Start with image number " << imageNumber << std::endl;
      }
    }
  }

  if(Utility::createDirectory(outputDirectory))
  {
    LOG(INFO) << tag << "Successfully created directory for captured disparity maps." << std::endl;
  }
  else
  {
    LOG(ERROR) << tag << "Unable to create directoryfor captured disparity maps." <<std::endl;
    return 0;
  }

  Stereosystem stereo(left,right);

  if(!stereo.loadIntrinsic(inputParameter+"/intrinsic.yml"))
  return 0;
  if(!stereo.loadExtrinisic(inputParameter +"/extrinsic.yml"))
  return 0;

  disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
  //disparityBM  = cv::StereoBM(CV_STEREO_BM_BASIC, numDispBM, windSizeBM);
   stereo.getRectifiedImagepair(s);

    R = stereo.getRotationMatrix();
    Q = stereo.getQMatrix();

    R.convertTo(R_32F,CV_32F);
    Q.convertTo(Q_32F,CV_32F);

    std::cout<<Q_32F<<std::endl;
    int key = 0;
    int binning =0;

    cv::Mat normalizedSGBM;
    cv::Mat leftColor;
    while(running)
    {

        stereo.getRectifiedImagepair(s);


        cvtColor(s.mLeft, leftColor, CV_GRAY2RGB);
        cv::putText(leftColor,qText.c_str(), cv::Point(10,80),CV_FONT_HERSHEY_PLAIN,2,CV_RGB(255,128,0));

        cv::imshow("Left", leftColor);
//        cv::imshow("Right", s.mRight);

        Disparity::sgbm(s, dispMapSGBM, disparitySGBM);
        cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
        cv::imshow("SGBM",normalizedSGBM);

        key = cv::waitKey(10);

        if(char(key) == 'q')
        {
            running = false;
        }
        else if(char(key) == 'b')
        {
            if (binning == 0)
            {
                binning = 1;
                fx = left->getIntrinsic().at<double>(0,0)/2.0;
                fy = left->getIntrinsic().at<double>(1,1)/2.0;
                cx = left->getIntrinsic().at<double>(0,2)/2.0;
                cy = left->getIntrinsic().at<double>(1,2)/2.0;
                scale=2;
            }
            else
            {
                binning =0;
                fx = left->getIntrinsic().at<double>(0,0);
                fy = left->getIntrinsic().at<double>(1,1);
                cx = left->getIntrinsic().at<double>(0,2);
                cy = left->getIntrinsic().at<double>(1,2);
                scale=1;
            }

            left->setBinning(binning);
            right->setBinning(binning);
            stereo.resetRectification();

        }
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

            {
                cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
                cv::imwrite(std::string(outputDirectory+"/sgbm_raw_"+prefix+std::to_string(imageNumber)+".jpg"),dispMapSGBM);
                cv::imwrite(std::string(outputDirectory+"/sgbm_norm_"+prefix+std::to_string(imageNumber)+".jpg"),normalizedSGBM);
            }
            ++imageNumber;
        }
        else if(char(key) == 'f')
            std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;

  }
  return 0;
}
