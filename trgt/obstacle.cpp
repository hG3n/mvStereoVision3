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
cv::Ptr<cv::StereoBM> disparityBM;

int minDisp, numDisp, blockSize, speckleWindowSize, speckleRange, disp12MaxDiff, preFilterCap, uniquenessRatio;
int disparityMode;
bool newDisparityMap = false;
bool reload = false;
int view;

// cv::Size imageSize(752, 480);

cv::Mat dMapRaw;
cv::Mat dMapNorm;
cv::Mat dMapWork(480, 752, CV_32F);

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
    newDisparityMap = true;
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

    fs["minDisp"]             >> minDisp;
    fs["numDisp"]             >> numDisp;
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

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void createDMapROIS(cv::Mat const& reference, cv::Rect & roi_u, cv::Rect& roi_b, int binning, bool reload = false)
{
  /* TODO:
      FIX THE STILL THERE RELOAD BUG
  */

  // the best value to work with in order to include everything and dont exclude important work
  int pixelShift = numDisp / 2;
  if(pixelShift % 2 == 1) {
    pixelShift = pixelShift + 1;
    if((reference.cols - pixelShift) % 8 != 0) {
      pixelShift = pixelShift + (reference.cols - pixelShift % 8);
    }
  }

  if(!reload) {
    roi_u = cv::Rect(cv::Point((pixelShift),0),cv::Point(reference.cols,reference.rows));
    roi_b = cv::Rect(cv::Point((pixelShift)/2,0),cv::Point(reference.cols/2,reference.rows/2));
  } else {
    if(binning == 1) {
      roi_u = cv::Rect(cv::Point((numDisp/3+blockSize)/2,0),cv::Point(reference.cols/2,reference.rows/2));
      roi_b = cv::Rect(cv::Point((numDisp/3+blockSize),0),cv::Point(reference.cols,reference.rows));
    } else {
      roi_u = cv::Rect(cv::Point((numDisp/3+blockSize),0),cv::Point(reference.cols,reference.rows));
      roi_b = cv::Rect(cv::Point((numDisp/3+blockSize)*2,0),cv::Point(reference.cols*2,reference.rows*2));
    }
  } 
}

void createSamplepointArray(std::vector<Samplepoint>& toFill, cv::Mat const& reference)
{
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
  disparitySGBM = cv::StereoSGBM::create(minDisp,
                                         numDisp,
                                         blockSize,
                                         8*blockSize*blockSize,
                                         32*blockSize*blockSize);
  std::thread disparity(disparityCalcSGBM,std::ref(s),std::ref(disparitySGBM));

  // load disparity parameters to get intial values
  if(!loadDisparityParameters("./configs/sgbm.yml"))
    return 0;

  // pre create the rois for binned and unbinned mode
  cv::Rect dMapROI_b, dMapROI_u;
  createDMapROIS(dMapWork, dMapROI_u, dMapROI_b, 0);

  // create subimage container to save created subdivisions
  std::vector<std::vector<cv::Mat>> subimages;
  subimages.reserve(9);

  // init obstacle detection
  obstacleDetection obst;
  std::vector<float> distanceMap;

  // test samplepoint distribution
  std::vector<Samplepoint> samplepoint_storage_u, samplepoint_storage_b;
  createSamplepointArray(samplepoint_storage_u, dMapWork(dMapROI_u));
  createSamplepointArray(samplepoint_storage_b, dMapWork(dMapROI_b));

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
      // printf("binning: %i cols: %i rows: %i \n",binning, dMapRaw.cols, dMapRaw.rows );

      // cut the disparity map in order to ignore the camera shift
      // camera shift is roughly calculated by a third of numDisparity
      // added up with the used blocksize
      if(reload) {
        createDMapROIS(dMapRaw, dMapROI_u, dMapROI_b, binning, reload);
        // std::cout << "recreated rois" << std::endl;
        // std::cout << "binned roi:   " << dMapROI_b << std::endl;
        // std::cout << "unbinned roi: " << dMapROI_u << std::endl;
        reload = false;
      }

      if(binning == 0) {
        dMapRaw.convertTo(dMapWork, CV_32F);
        dMapWork = dMapRaw(dMapROI_u);
      } else if (binning == 1) {
        dMapRaw.convertTo(dMapWork, CV_32F);
        dMapWork = dMapRaw(dMapROI_b);
      }

      // calculate samplepoint value and draw samplepoints
      

      // display samplepoint for debug purpose
      // std::for_each(samplepoint_storage.begin(), samplepoint_storage.end(), [](Samplepoint s){
      //   std::cout << s.center << std::endl;
      // });

      // clear current subimages and refill the container with new ones
      // subimages.clear();
      // subdivideImages(dMapWork, subimages, binning);
      // obst.buildMeanDistanceMap(Q_32F, subimages, binning);

      // display stuff
      cv::normalize(dMapWork,dMapNorm,0,255,cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(dMapNorm,dMapNorm,CV_GRAY2BGR);

      // if(binning == 0) {
      //   std::for_each(samplepoint_storage_u.begin(), samplepoint_storage_u.end(), [](Samplepoint s){
      //     s.calculateSamplepointValue(dMapNorm);
      //   });
      // } else {
      //   std::for_each(samplepoint_storage_b.begin(), samplepoint_storage_b.end(), [](Samplepoint s){
      //     s.calculateSamplepointValue(dMapNorm);
      //   });
      // }

      if (view % 3 == 0)
      {
         View::drawSubimageGrid(dMapNorm, binning);
         View::drawObstacleGrid(dMapNorm, binning);
      } else if (view % 3 == 1) {
        if(binning == 0) {
          std::for_each(samplepoint_storage_u.begin(), samplepoint_storage_u.end(), [](Samplepoint s){
            s.drawSamplepoints(dMapNorm);
          });
        } else {
          std::for_each(samplepoint_storage_b.begin(), samplepoint_storage_b.end(), [](Samplepoint s){
            s.drawSamplepoints(dMapNorm);
          });
        }
      }

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
        {
          if (binning == 0)
            binning = 1;
          else
            binning = 0;
          left->setBinning(binning);
          right->setBinning(binning);
          stereo.resetRectification();
          break;
        }
        case 'f':
          std::cout<< left->getFramerate() << " " << right-> getFramerate() <<std::endl;
          break;
        case 'r':
          loadDisparityParameters("./configs/sgbm.yml");
          reload = true;
          break;
        case '0':
          obst.detectObstacles(0, std::make_pair(0.8, 1.2));
          reload = true;
          std::cout << "" << std::endl;
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
        case 's':
          std::cout << dMapWork.size() << std::endl;
          // std::cout << (dMapWork.cols + (8-(dMapWork.cols % 8))) % 8 << std::endl;
          std::cout << dMapWork.cols%8 << std::endl;
          std::cout << dMapWork.rows%8 << std::endl;
          break;
        case 'n':
        {
          cv::FileStorage g("afterCalibrationParameters.yml", cv::FileStorage::WRITE);
          g << "Q" << Q_32F;
          g << "K_L" << stereo.getNewKMats()[0];
          g << "K_R" << stereo.getNewKMats()[1];
          break;
        }
        case 'd':
          {
            int c = dMapWork.cols/2;
            int r = dMapWork.rows/2;
            Samplepoint s(cv::Point(c,r), 1);
            s.calculateSamplepointValue(dMapWork);
            std::cout << dMapWork.at<short>(r,c) << std::endl;
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
