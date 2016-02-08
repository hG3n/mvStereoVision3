#include <iostream>
#include <string>
#include <chrono>
#include <ctime>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>

//system command
#include <cstdlib>

// user includes
#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"
#include "view.h"

#include "ObstacleDetection.h"
#include "MeanDisparityDetection.h"
#include "SamplepointDetection.h"

#include "Samplepoint.h"
#include "Subimage.h"

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
std::vector<Subimage> found;

cv::Mat Q, Q_32F;
cv::Mat R, R_32F;

int exposure = 10000;
float gain = 4.0;
bool hdr = false;

// framerate counter disparity map
int dmap_counter = 0;
std::vector<float> dmap_times;

void disparityCalcSGBM(Stereopair const& s, cv::Ptr<cv::StereoSGBM> disparity)
{
  while(running)
  {
    // measure time for each thread loop to be completed
    clock_t start = clock();


    std::unique_lock<std::mutex> ul(disparityLock);
    cond_var.wait(ul);
    Disparity::sgbm(s, dMapRaw, disparity);
    newDisparityMap = true;

    // get end time
    clock_t end = clock();
    float seconds = (float)(end-start)/CLOCKS_PER_SEC;
    dmap_times.push_back(seconds);

    ++dmap_counter;
  }
}

void mouseClick(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN ) {
    float d = static_cast<float>(dMapRaw.at<short>(y,x));
    printf("Disparity at: (%i,%i) = %f\n",x,y,d);
  }
}

void createDMapROIS(cv::Mat const& reference, cv::Rect & roi, bool reload = false)
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
    roi = cv::Rect(cv::Point((pixelShift),0),cv::Point(reference.cols,reference.rows));
  }
}


void drawFoundObstacles(cv::Mat& reference, std::vector<Subimage> const& obstacle_vec)
{
  std::for_each(obstacle_vec.begin(), obstacle_vec.end(), [&reference] (Subimage s){
    cv::circle(reference, s.roi_center, 3, cv::Scalar(0,0,255), -1);
  });
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
  left->setBinning(Camera::binning::BINNING_HV);
  right->setBinning(Camera::binning::BINNING_HV);
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
  int binning = 1;
  float factor = 0.5;
  // get one single imagepair in order to receive the Q Matrix
  // which is created during the rectification
  // multiply q mat with the size factor of the captured image
  // in order o to get the correct dimensions
  stereo.getRectifiedImagepair(s, factor);
  Q = stereo.getQMatrix();
  Q.convertTo(Q_32F,CV_32FC1);
  Q_32F = Q_32F * factor;
  Q_32F.at<float>(3,2) = Q_32F.at<float>(3,2) / 0.5;
  Q_32F.at<float>(2,3) = Q_32F.at<float>(2,3) / 0.5;
  std::cout << Q_32F << std::endl;

  // init OpenCV GUI objects
  // cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  // cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  // cv::namedWindow("SGBM" ,1);
  // cv::setMouseCallback("SGBM", mouseClick, NULL);

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
  cv::Rect dMapROI;
  std::cout << dMapWork.size() << std::endl;
  createDMapROIS(dMapWork, dMapROI);
  std::cout << dMapROI << std::endl;

  // init obstacle detection 
  ObstacleDetection *o;
  MeanDisparityDetection m;
  bool detectionIsInit = true;
  m.init(dMapWork,Q_32F, 0.1, 1.5);
  o = &m;

  running = true;
  int frame = 0;
  int detection_frame = 0;

  std::vector<float> times;
  while(running)
  {
    stereo.getRectifiedImagepair(s, 0.5);
    // cv::imshow("Left", s.mLeft);
    // cv::imshow("Right", s.mRight);

    // notify the thread to start 
    cond_var.notify_one();

    if(newDisparityMap)
    {
      // cut the disparity map in order to ignore the camera shift
      // camera shift is roughly calculated by a third of numDisparity
      // added up with the used blocksize
      // if(reload) {
      //   std::cout << "dMapRaw size: " << dMapRaw.size() << std::endl;
      //   createDMapROIS(dMapRaw, dMapROI, reload);
      //   reload = false;
      // }

      dMapRaw.convertTo(dMapWork, CV_32F);
      // dMapWork = dMapRaw(dMapROI);
      dMapWork = dMapRaw;

      if(detectionIsInit){
        Q = stereo.getQMatrix();
        Q.convertTo(Q_32F,CV_32FC1);

        m.init(dMapWork,Q_32F, 0.1, 1.5);
        o = &m;
        detectionIsInit = false;
      }

      o->build(dMapWork, binning, 1);
      clock_t start = clock();
      o->detectObstacles();
      clock_t end = clock();
      float seconds = (float)(end-start)/CLOCKS_PER_SEC;
      times.push_back(seconds);

      found = m.getFoundObstacles();

      // display stuff
      cv::normalize(dMapWork,dMapNorm,0,255,cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(dMapNorm,dMapNorm,CV_GRAY2BGR);

      // visualization
      if (view % 3 == 0) {
        // TODO fix that!!!
        // std::for_each(layout_container.begin(), layout_container.end(), [&dMapNorm, layout_container] (Samplepoint s) {
        //   cv::rectangle(dMapNorm, s.roi.tl, s.roi.br, cv::Scalar(0,0,255));
        // });
      } else {
        drawFoundObstacles(dMapNorm, found);
      }

      // cv::imshow("SGBM",dMapNorm);
      newDisparityMap = false;
      ++detection_frame;
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
        case 'h':
          if(!hdr) {
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
          // Utility::dmap2pcl("pointcloud.ply", dMapRaw, Q_32F);
          o->detectObstacles();
          std::cout << " " << std::endl;
          break;
        case 'p':
          std::cout << Q_32F << std::endl;
          break;
        case 'v':
            ++view;
            break;
        default:
          std::cout << "Key pressed has no action" << std::endl;
          break;
      }
    }
    ++frame;

     if(detection_frame == 1000){
       printf("%i\n", detection_frame);
       float total = 0.0f;
       std::for_each(times.begin(), times.end(), [&total](float value){
         total += value;
       });
       float average = total/times.size();
       printf("average computation time: %f\n", average);
       printf("Detection Framerate: %f\n", 1/average);
       printf("\n");
       times.clear();
       times.resize(0);
     }

    if(dmap_counter == 1000){
      std::cout << "dmap_counter: " << dmap_counter << std::endl;
      float total = 0.0f;
      std::for_each(dmap_times.begin(), dmap_times.end(), [&total](float value) {
        total += value;
      });
      std::cout << total << " / " << dmap_times.size() << std::endl;
      float average = total/dmap_times.size();
      std::cout << "average computation time: " << average << std::endl;
      std::cout << "Disparity Framerate: " << 1/average << std::endl;
      std::cout << "vector size: " << dmap_times.size() << std::endl;
      std::cout << std::endl;
      dmap_times.clear();
      dmap_times.resize(0);
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
