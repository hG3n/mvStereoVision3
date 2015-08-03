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

cv::Rect viewAreaLeft, viewAreaRight;
cv::Point p1Left,p2Left, p1Right, p2Right;
int clickLeft=3;
int clickRight=3;

void mouseClickLeft(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN )
  {
    clickLeft++;
    if(clickLeft % 3 == 1)
      p1Left = cv::Point(x,y);
    else if(clickLeft % 3 == 2)
    {
      p2Left = cv::Point(x,y);
      viewAreaLeft = cv::Rect(p1Left,p2Left);
    }
    else if(clickLeft % 3 == 0) 
    {
      p1Left = cv::Point(0,0);
      p2Left = cv::Point(0,0);
      viewAreaLeft = cv::Rect(p1Left,p2Left);
    }
  }
}

void mouseClickRight(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN )
  {
    clickRight++;
    if(clickRight % 3 == 1)
      p1Right = cv::Point(x,y);
    else if(clickRight % 3 == 2)
    {
      p2Right = cv::Point(x,y);
      viewAreaRight = cv::Rect(p1Right,p2Right);
    }
    else if(clickRight % 3 == 0) 
    {
      p1Right = cv::Point(0,0);
      p2Right = cv::Point(0,0);
      viewAreaRight = cv::Rect(p1Right,p2Right);
    }
  }
}

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
  Stereopair s;

  left->setExposure(10000);
  right->setExposure(10000);

  double sharpnessLeft = 0;
  double sharpnessRight = 0;

  char key = 0;
  int binning = 0;
  cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  bool running = true;
  std::string filename = "";
  while(running)
  {

    if(!stereo.getImagepair(s))
    {
      LOG(ERROR) << tag << "Unable to get imagepair" << std::endl;
      break;
    }
    cv::imshow("Left", s.mLeft);
    cv::imshow("Right", s.mRight);
    cv::setMouseCallback("Left", mouseClickLeft, NULL);
    cv::setMouseCallback("Right", mouseClickRight, NULL);

    if(viewAreaLeft.size().width != 0 && viewAreaLeft.size().height != 0) {
      cv::imshow("rectleft", s.mLeft(viewAreaLeft));
      sharpnessLeft = Utility::checkSharpness(s.mLeft(viewAreaLeft)); 
    }

    if(viewAreaRight.size().width != 0 && viewAreaRight.size().height != 0) {
      cv::imshow("rectRight", s.mRight(viewAreaRight));
      sharpnessRight = Utility::checkSharpness(s.mRight(viewAreaRight)); 
    }

    std::cout << "sharpness left: " << sharpnessLeft << "\t\t sharpnessRight: " << sharpnessRight << std::endl;

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
