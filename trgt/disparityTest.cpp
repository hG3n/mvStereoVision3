#include <iostream>
#include <string>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#include <cmath>

INITIALIZE_EASYLOGGINGPP

cv::StereoSGBM disparitySGBM;
cv::StereoBM disparityBM;

int numDispSGBM = 16;
int windSizeSGBM = 5;

int numDispBM = 16;
int windSizeBM = 5;

double baseline ;
double fx ;
double fy ;
double cx ;
double cy ;
cv::Mat dispMapSGBM, dispMapBM, dispMapTM;

cv::Mat R, R_32F;
cv::Mat disparityMap_32FC1;


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

void changeNumDispBM(int, void*)
{
    numDispBM+=numDispBM%16;

    if(numDispBM < 16)
    {
        numDispBM = 16;
        cv::setTrackbarPos("Num Disp", "BM", numDispBM);
    }
    cv::setTrackbarPos("Num Disp", "BM", numDispBM);
    disparityBM = cv::StereoBM(CV_STEREO_BM_BASIC, numDispBM, windSizeBM);
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

void changeWindSizeBM(int, void*)
{
    if(windSizeBM%2 == 0)
        windSizeBM+=1;


    if(windSizeBM < 5)
    {
        windSizeBM = 5;
        cv::setTrackbarPos("Wind Size", "BM", windSizeBM);
    }
    cv::setTrackbarPos("Wind Size", "BM", windSizeBM);
    disparityBM = cv::StereoBM(CV_STEREO_BM_BASIC, numDispBM, windSizeBM);
}

void mouseClick(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN )
     {
            double d = double(disparityMap_32FC1.at<float>(x,y));

            if(d > 0)
            {
                cv::Mat_<float>  p(3,1), pFinish(3,1);
                p(2) = baseline*fx/d;
                p(0) = p(2)*(y-cx)/fx;
                p(1) = p(2)*(x-cy)/fy;
                pFinish = R_32F.t()*p;
                std::cout<< pFinish(0) << " " << pFinish(1) << " " << pFinish(2) << std::endl;
                std::cout<<"distance: " << sqrt(pow(p(0),2)+pow(p(1),2)+pow(p(2),2)) <<std::endl;
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
     // else if ( event == CV_EVENT_MOUSEMOVE )
     // {
     //      std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;

     // }


}

int main(int argc, char* argv[])
{
    std::string tag = "MAIN\t";

    LOG(INFO) << tag << "Application started.";

  std::vector<std::string> nodes;
  nodes.push_back("inputParameter");
  nodes.push_back("capturedDisparities");

  std::string config = "./configs/default.yml";

  cv::FileStorage fs;

    baseline = 255.752069442;
    fx = 614.245/2.0;
    fy = 614.456/2.0;
    cx = 376.9866/2.0;
    cy = 230.39104/2.0;

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

    int key = 0;
    Stereopair s;

    cv::Mat left = cv::imread("./images/rectifiedImages/left/left_0010.jpg");
    cv::Mat right = cv::imread("./images/rectifiedImages/right/right_0010.jpg");

    cv::resize(left, left, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
    cv::resize(right, right, cv::Size(), 0.5, 0.5, CV_INTER_AREA);

    cv::Mat grayLeft, grayRight;
    cv::cvtColor(left, grayLeft, CV_BGR2GRAY);
    cv::cvtColor(right, grayRight, CV_BGR2GRAY);
    s.mLeft = grayLeft;
    s.mRight = grayRight;

    bool success = fs.open("./parameter/extrinsic.yml", cv::FileStorage::READ);

    if(fs["R"].empty() || fs["T"].empty() || fs["E"].empty() || fs["F"].empty())
    {
        LOG(ERROR) << tag << "Node is empty." <<std::endl;
        fs.release();
        return 0;
    }

  if(success)
  {
      fs["R"] >> R;
      LOG(INFO) << tag <<"Successfully loaded Extrinsics." << std::endl;
      fs.release();
    }
  else
  {
    LOG(ERROR) << tag << "Unable to open extrinsic file: " << std::endl;
      fs.release();
    return 0;
  }

    R.convertTo(R_32F,CV_32F);


    disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
    disparityBM = cv::StereoBM(CV_STEREO_BM_BASIC, numDispBM, windSizeBM);

    cv::namedWindow("SGBM" ,1);
    cv::namedWindow("BM" ,1);

    cv::createTrackbar("Num Disp", "SGBM", &numDispSGBM, 320, changeNumDispSGBM);
    cv::createTrackbar("Wind Size", "SGBM", &windSizeSGBM, 51, changeWindSizeSGBM);

    cv::createTrackbar("Num Disp", "BM", &numDispBM, 320, changeNumDispBM);
    cv::createTrackbar("Wind Size", "BM", &windSizeBM, 51, changeWindSizeBM);

    cv::setMouseCallback("SGBM", mouseClick, NULL);


    cv::Mat normalizedSGBM;
    cv::Mat normalizedBM;
    cv::Mat normalizedTM;


    bool running = true;

    while(running)
    {


        cv::imshow("Left", s.mLeft);
        cv::imshow("Right", s.mRight);

        Disparity::sgbm(s, dispMapSGBM, disparitySGBM);
        Disparity::bm(s, dispMapBM, disparityBM);
  //      Disparity::tm(s, dispMapTM,5);

         dispMapSGBM.convertTo(disparityMap_32FC1,CV_32FC1);

       //  cv::threshold(dispMapSGBM,dispMapSGBM,1.0,300,CV_THRESH_TOZERO);
         cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
         //cv::resize(normalizedSGBM, normalizedSGBM, cv::Size(), 2,2, CV_INTER_AREA);
         cv::imshow("SGBM",normalizedSGBM);

         //cv::normalize(dispMapBM,normalizedBM,0,255,cv::NORM_MINMAX, CV_8U);
         //cv::resize(normalizedBM, normalizedBM, cv::Size(), 2,2, CV_INTER_AREA);
         //cv::imshow("BM",normalizedBM);

        // cv::normalize(dispMapTM,normalizedTM,0,255,cv::NORM_MINMAX, CV_8U);
        // cv::imshow("TM",normalizedTM);

        key = cv::waitKey(10);


        if(char(key) == 'q')
        {
            running = false;
            break;
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

                cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
                cv::imwrite(std::string(outputDirectory+"/sgbm_raw_nd"+ std::to_string(numDispSGBM)+"_ws"+std::to_string(windSizeSGBM)+"_"+prefix+std::to_string(imageNumber)+".jpg"),dispMapSGBM);
                cv::imwrite(std::string(outputDirectory+"/sgbm_norm_nd"+ std::to_string(numDispSGBM)+"_ws"+std::to_string(windSizeSGBM)+"_"+prefix+std::to_string(imageNumber)+".jpg"),normalizedSGBM);

                cv::normalize(dispMapBM,normalizedBM,0,255,cv::NORM_MINMAX, CV_8U);
                cv::imwrite(std::string(outputDirectory+"/bm_raw_nd"+ std::to_string(numDispBM)+"_ws"+std::to_string(windSizeBM)+"_"+prefix+std::to_string(imageNumber)+".jpg"),dispMapBM);
                cv::imwrite(std::string(outputDirectory+"/bm_norm_nd"+ std::to_string(numDispBM)+"_ws"+std::to_string(windSizeBM)+"_"+prefix+std::to_string(imageNumber)+".jpg"),normalizedBM);

                // cv::normalize(dispMapTM,normalizedTM,0,255,cv::NORM_MINMAX, CV_8U);
                // cv::imwrite(std::string(outputDirectory+"/tm_raw_"+prefix+std::to_string(imageNumber)+".jpg"),dispMapTM);
                // cv::imwrite(std::string(outputDirectory+"/tm_norm_"+prefix+std::to_string(imageNumber)+".jpg"),normalizedTM);

            ++imageNumber;
        }
    }
    return 0;
}
