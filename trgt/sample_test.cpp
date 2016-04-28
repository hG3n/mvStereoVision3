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
#include "SamplepointDetection.h"

#include "Samplepoint.h"

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
std::vector<Samplepoint> found;

cv::Mat Q, Q_32F;
cv::Mat R, R_32F;

int exposure = 10000;
float gain = 4.0;
bool hdr = false;

ObstacleDetection *o;
SamplepointDetection sd;
bool detectionIsInit = true;

int test_nr = 0;
std::string test_name;

void disparityCalcSGBM(Stereopair const& s, cv::Ptr<cv::StereoSGBM> disparity)
{
  while(running)
  {
    std::unique_lock<std::mutex> ul(disparityLock);
    cond_var.wait(ul);
    Disparity::sgbm(s, dMapRaw, disparity);
    newDisparityMap = true;
  }
}

void mouseClick(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN ) {
    float d = static_cast<float>(dMapRaw.at<short>(y,x));
    printf("Disparity at: (%i,%i) = %f\n",x,y,d);
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


void drawFoundObstacles(cv::Mat& reference, std::vector<Samplepoint> const& obstacle_vec)
{
  std::for_each(obstacle_vec.begin(), obstacle_vec.end(), [&reference] (Samplepoint s){
    cv::circle(reference, s.center, 3, cv::Scalar(0,0,255), -1);
  });
}

bool save_test_set(std::vector<Samplepoint> const& found_obstacles, Stereopair const& s, int binning)
{
  std::cout << "saving test set nr: " << test_nr << std::endl;

  std::string directory = "test/"+test_name+"/";
  if(!(Utility::createDirectory(directory))) {
    LOG(INFO) << "failed to create test directory!" << std::endl;
    return false;
  }

  // save full pointcloud
  std::cout << "saving full pointcloud" << std::endl;
  Utility::dmap2pcl(directory+ "test_pcl_" + std::to_string(test_nr)+".ply", dMapWork, Q_32F);

  // save images
  cv::imwrite(directory+"_test_"+ std::to_string(test_nr) +"_left.jpg",s.mLeft);
  cv::imwrite(directory+"_test_"+ std::to_string(test_nr) +"_right.jpg",s.mRight);
  cv::imwrite(directory+"_test_"+ std::to_string(test_nr) +"_disparity.jpg",dMapNorm);

  // save disparity matrix
  cv::FileStorage fs(directory+"_test_"+std::to_string(test_nr)+"_dmaps.yml",cv::FileStorage::WRITE);
  fs << "dMapRaw" << dMapRaw;
  fs << "dMapWork" << dMapWork;
  fs.release();

  // call obstacle detection and get newest found vector
  o->build(dMapWork, binning,0);
  o->detectObstacles();
  found = sd.getFoundObstacles();
  int obstacle_frame = sd.getObstacleCounter();

  // save found obstacles to csv
  std::ofstream out;
  out.open(directory+test_name+"_test_"+ std::to_string(test_nr) + "_found_obstacles.csv");
  out << "center_x,center_y,radius,roi_tl_x,roi_tl.y,roi_br_x,roi_br_y,value\n";
  std::for_each(found_obstacles.begin(), found_obstacles.end(), [&out] (Samplepoint s) {
    out << s.center.x << "," << s.center.y << ","
        << s.radius << ","
        // << s.roi.tl << "," << s.roi.tl << ","
        // << s.roi.br << "," << s.roi.br << ","
        << s.value << "\n";
  });

  // save general infos to csv
  int real_world_distance;
  std::cout << "testing distance:           ";
  std::cin >> real_world_distance;
  std::cout << "" << std::endl;


  std::ofstream test_general;
  test_general.open(directory+"general_"+test_name + "_" + std::to_string(test_nr)+".csv");
  test_general << "testnr,obstacle_frame,testing_distance,num_found\n";
  test_general << test_nr << "," << obstacle_frame << "," << real_world_distance << "," << found_obstacles.size() << "\n";
  ++test_nr;
return true;
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
  cv::setMouseCallback("SGBM", mouseClick, NULL);


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

  // create subimage container to save created subdivisions
  sd.init(dMapWork, Q_32F, 0.1, 2.0);
  std::vector<Samplepoint> layout_container = sd.getFoundObstacles();
  o = &sd;

  std::cout << "test_name: ";
  std::cin >> test_name;
  std::cout << "" << std::endl;

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
          Q.convertTo(Q_32F,CV_32FC1);
          
          sd.init(dMapWork, Q_32F, 0.1, 2.0);
          o = &sd;
          detectionIsInit = false;
        }

      } else if (binning == 1) {
        dMapRaw.convertTo(dMapWork, CV_32F);
        dMapWork = dMapRaw(dMapROI_b);
        if(detectionIsInit) {
          Q = stereo.getQMatrix();
          Q.convertTo(Q_32F,CV_32FC1);
         
          sd.init(dMapWork, Q_32F, 0.1, 2.0);
          o = &sd;
          detectionIsInit = false;
        }
      }

      // obstacle detection
      o->build(dMapWork, binning, 0);
      o->detectObstacles();
      found = sd.getFoundObstacles();

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
          Utility::dmap2pcl("pointcloud.ply", dMapRaw, Q_32F);
          cv::imwrite("dmap_norm.png", dMapNorm);
          break;
        case 'p':
          std::cout << Q_32F << std::endl;
          break;
        case 't':
          if(!(save_test_set(found, s, binning)))
            continue;
          break;
        case 's':
        {
          cv::imwrite("dmap_" + std::to_string(frame) + ".jpg", dMapNorm);
          cv::Mat dMapRawNorm;
          cv::normalize(dMapRaw,dMapRawNorm,0,255,cv::NORM_MINMAX, CV_8U);
          cv::imwrite("dmap_full_" + std::to_string(frame) + ".jpg", dMapRawNorm);
          break;
        }
        case 'v':
            ++view;
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
