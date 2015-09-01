#include "disparity.h"
#include "utility.h"
#include "math.h"

// Semi Global Block matching 
void Disparity::sgbm(Stereopair const& inputImages, cv::Mat &output, cv::Ptr<cv::StereoSGBM> dispCompute)
{
  dispCompute->compute(inputImages.mLeft,inputImages.mRight,output);
	//output/=16;
}

// void Disparity::binary_sgbm(Stereopair const& inputImages, cv::Mat &output, cv::Ptr<cv::StereoSGBM> dispCompute)
// {
//   dispCompute->compute(inputImages.mLeft,inputImages.mRight,output);
// }

// simple blockmatching 
void Disparity::bm(Stereopair const& inputImages, cv::Mat &output, cv::Ptr<cv::StereoBM> dispCompute)
{
  dispCompute->compute(inputImages.mLeft,inputImages.mRight,output);
  //output/=16;
}

// self written template matching algorithm
void Disparity::tm(Stereopair const& inputImages, cv::Mat &output, unsigned int kernelSize)
{
  output = cv::Mat(inputImages.mLeft.rows, inputImages.mLeft.cols,CV_8U,cv::Scalar::all(0));

  cv::Mat currentTemplate;
  cv::Mat currentSearchRange;
  cv::Mat result;

  double minVal;
  double maxVal;
  cv::Point2i minLoc;
  cv::Point2i maxLoc;
  // cv::Point2i matchLoc;

  for(unsigned int i = 0; i < inputImages.mLeft.rows-kernelSize; ++i)
  {
    for(unsigned int j = 0; j < inputImages.mLeft.cols-kernelSize ; ++j)
    {
      cv::Point2i position(j,i);
      cv::Rect kernel(position, cv::Size(kernelSize,kernelSize));

      currentTemplate = inputImages.mLeft(kernel);
      int destWidth = inputImages.mRight.cols - position.x-1;
      currentSearchRange = inputImages.mRight(cv::Rect(position,cv::Size(destWidth,kernelSize)));

      cv::matchTemplate(currentSearchRange,currentTemplate,result,CV_TM_CCORR_NORMED);

      cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

      output.at<char>(i,j) = maxLoc.x;
      //std::cout<< int(output.at<char>(i,j)) <<std::endl;
    }
  }
}

bool Disparity::loadSGBMParameters(std::string const filename, cv::Ptr<cv::StereoSGBM> & disparityObj, sgbmParameters & para)
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

    fs["minDisp"]             >> para.minDisp;
    fs["numDisp"]             >> para.numDisp;
    fs["blockSize"]           >> para.blockSize;
    fs["disp12MaxDiff"]       >> para.disp12MaxDiff;
    fs["preFilterCap"]        >> para.preFilterCap;
    fs["uniquenessRatio"]     >> para.uniquenessRatio;
    fs["speckleWindowSize"]   >> para.speckleWindowSize;
    fs["speckleWindowRange"]  >> para.speckleRange;
    fs["mode"]                >> para.disparityMode;

    disparityObj->setMinDisparity(para.minDisp);
    disparityObj->setNumDisparities(para.numDisp);
    disparityObj->setBlockSize(para.blockSize);
    disparityObj->setPreFilterCap(para.preFilterCap);
    disparityObj->setUniquenessRatio(para.uniquenessRatio);
    disparityObj->setDisp12MaxDiff(para.disp12MaxDiff);
    disparityObj->setSpeckleWindowSize(para.speckleWindowSize);
    disparityObj->setSpeckleRange(para.speckleRange);

    if(para.disparityMode == 1 )
      disparityObj->setMode(cv::StereoSGBM::MODE_HH);
    else
      disparityObj->setMode(cv::StereoSGBM::MODE_SGBM);
  
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