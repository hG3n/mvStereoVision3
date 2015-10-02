#include "utility.h"

std::string mTag = "UTILITY\t";


// -----------------------------------------------------------------------------
// --- streopair ---------------------------------------------------------------
// -----------------------------------------------------------------------------
Stereopair::Stereopair():
  mLeft(),
  mRight(),
  mTag("STEREOPAIR\t")
{}

Stereopair::Stereopair(cv::Mat &l , cv::Mat &r):
	mLeft(l),
	mRight(r),
  mTag("STEREOPAIR\t")
{}

Stereopair::~Stereopair()
{
  mLeft.release();
  mRight.release();
  LOG(INFO) << mTag << "Stereopair destroyed" << std::endl;
}


// -----------------------------------------------------------------------------
// --- utility - directory -----------------------------------------------------
// -----------------------------------------------------------------------------
int Utility::getFiles (std::string const& dir, std::vector<std::string> &files)
{
  DIR *dp;
  struct dirent *dirp;

  //Unable to open dir
  if((dp  = opendir(dir.c_str())) == NULL)
  {
    std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    return errno;
  }

  //read files and push them to vector
  while ((dirp = readdir(dp)) != NULL)
  {
    std::string name = std::string(dirp->d_name);
    //discard . and .. from list
    if(name != "." && name != "..")
    {
      files.push_back(std::string(dirp->d_name));
    }
  }

  closedir(dp);
  std::sort(files.begin(), files.end());

  return 0;
}

bool Utility::directoryExist(std::string const& dirPath)
{
	struct stat st = {0};
	if(stat(dirPath.c_str(),&st) == 0)
	{
		return true;
	}
	return false;
}

bool Utility::createDirectory(std::string const& dirPath)
{
  system(std::string("mkdir -p " + dirPath).c_str());
  if(directoryExist(dirPath))
    return true;
  else
    return false;
}


// -----------------------------------------------------------------------------
// --- utility - init Cameras --------------------------------------------------
// -----------------------------------------------------------------------------
bool Utility::initCameras(mvIMPACT::acquire::DeviceManager &devMgr, Camera *&left, Camera *&right)
{
  const unsigned int devCnt = devMgr.deviceCount();

  if(devCnt != 2)
  {
    LOG(ERROR)<< mTag <<"Invalid number of cameras detected! Number of detected cameras: " <<\
    devCnt << std::endl;
    return false;
  }


  if(devMgr[0]->serial.read() == "26803878")
  {
    if(devMgr[1]->serial.read() == "26803881")
    {
      LOG(INFO)<< mTag << "Successfully initialized both cameras" <<std::endl;

      left = new Camera(devMgr[0]);
      right= new Camera(devMgr[1]);
      return true;
    }
    LOG(ERROR)<< mTag << "Error in camera initialization, got Serials:" <<\
    devMgr[0]->serial.read()<< " " <<devMgr[1]->serial.read()<<std::endl;
    return false;
  }

  if(devMgr[0]->serial.read() == "26803881")
  {
    if(devMgr[1]->serial.read() == "26803878")
    {
      LOG(INFO)<< mTag << "Successfully initialized both camers" <<std::endl;

      left = new Camera(devMgr[1]);
      right = new Camera(devMgr[0]);
      return true;
    }
    LOG(ERROR)<< mTag << "Error in camera initialization, got Serials:" <<\
    devMgr[0]->serial.read()<< " " <<devMgr[1]->serial.read()<<std::endl;
    return false;
  }
  return false;
}


// -----------------------------------------------------------------------------
// --- utility - config --------------------------------------------------------
// -----------------------------------------------------------------------------
bool Utility::checkConfig(std::string const& configfile, std::vector<std::string> const& nodes, cv::FileStorage &fs)
{
  bool success = fs.open(configfile, cv::FileStorage::READ);

  for(std::string currentNode : nodes)
  {
    if(fs[currentNode].empty())
    {
      LOG(ERROR) << mTag << "Node " << currentNode << " in " << configfile << " is empty." << std::endl;
      fs.release();
      return false;
    }
  }

  if(success)
  {
    LOG(INFO) << mTag <<"Successfully checked configuration." << std::endl;;
    return true;
   }
  else
  {
    LOG(ERROR) << mTag << "Unable to open configuration: " << configfile << std::endl;
    fs.release();
    return false;
  }
}


// -----------------------------------------------------------------------------
// --- utility - helper --------------------------------------------------------
// -----------------------------------------------------------------------------
double Utility::checkSharpness(cv::Mat const& src)
{
  cv::Mat M = (cv::Mat_<double>(3, 1) << -1, 2, -1);
  cv::Mat G = cv::getGaussianKernel(3, -1, CV_64F);
 
  cv::Mat Lx, Ly;
  cv::sepFilter2D(src, Lx, CV_64F, M, G);
  cv::sepFilter2D(src, Ly, CV_64F, G, M);

  cv::Mat FM = cv::abs(Lx) + cv::abs(Ly);
  return cv::mean(FM).val[0];
}

cv::Mat Utility::calcCoordinate(cv::Mat const& Q, cv::Mat const& dMap,int x,int y)
{
  cv::Mat_<float> coordinate(1,4);
  float d = dMap.at<float>(x,y);
  d/=16.0;
  
  if(d > 0) {
    coordinate(0)=x;
    coordinate(1)=y;
    coordinate(2)=d;
    coordinate(3)=1;

    coordinate = cv::Mat(coordinate);

    coordinate = Q * coordinate.t();
    coordinate /= coordinate(3);
    
    return coordinate;
  } else {
    coordinate(0) = 0;
    coordinate(1) = 0;
    coordinate(2) = 0;
    coordinate(3) = 0;
    return coordinate;
  }
}

cv::Mat Utility::calcCoordinate(cv::Mat const& Q, float dValue, int x, int y)
{
  cv::Mat_<float> coordinate(1,4);
  dValue /= 16.0;
  
  if(dValue > 0) {
    coordinate(0)=x;
    coordinate(1)=y;
    coordinate(2)=dValue;
    coordinate(3)=1;

    coordinate = cv::Mat(coordinate);

    coordinate = Q * coordinate.t();
    coordinate /= coordinate(3);
    
    return coordinate;
  } else {
    coordinate(0) = 0;
    coordinate(1) = 0;
    coordinate(2) = 0;
    coordinate(3) = 0;
    return coordinate;
  }
}

float Utility::calcDistance(cv::Mat const& Q, float const& dispValue, int binning)
{
  float d = dispValue / 16;
  cv::Mat_<float> coordinateQ(1,4);

  coordinateQ(0)=1;
  coordinateQ(1)=1;
  coordinateQ(2)=d;
  coordinateQ(3)=1;

  if(binning == 0)
  {
    coordinateQ = (Q)*coordinateQ.t();
    coordinateQ/=coordinateQ(3);
  
    float distance = coordinateQ(2)/1000;
    coordinateQ.release();

    return distance;
  }
  else
  {
    coordinateQ = (Q/2)*coordinateQ.t();
    coordinateQ/=coordinateQ(3);
    
    float distance = coordinateQ(2)/1000;
    coordinateQ.release();

    // because binning is half of the image
    if(cvIsInf(distance))
    {
      return distance;
    }
    else
      return distance/2;
  }
}

float Utility::calcMeanDisparity(cv::Mat const& matrix)
{
  int total = 0;
  int numElements = 0;

  std::for_each(matrix.begin<short>(), matrix.end<short>(), [&numElements, &total](short value) {
    if(value > 1) {
      total += value;
      ++numElements;
    }
  });

  // if the matrix appears to be empty because of any reason
  // return disparity of 1.0 to indicate infinity
  if(total == 0 || numElements == 0){
    return 0.0;
  }
  else
  {
    float mean = total / abs(numElements);
    return mean;
  }
}

std::pair<float,float> Utility::calcMinMaxDisparity(cv::Mat const& matrix)
{
  std::vector<float> elements;
  for(int r = 0; r < matrix.rows; ++r)
  {
    for(int c = 0; c < matrix.cols; ++c)
    {
      if(static_cast<float>(matrix.at<short>(r,c)) > 0)
      {
        float current = static_cast<float>(matrix.at<short>(r,c));
        elements.push_back(current);
      }
    }
  } 
  auto min = std::min_element(std::begin(elements), std::end(elements));
  auto max = std::max_element(std::begin(elements), std::end(elements));
  return std::make_pair(*min,*max);
}

float Utility::calcStdDev(cv::Mat const& matrix)
{
  // function to calculate the stddev of valid disparity pixels within a matrix
  float mean = Utility::calcMeanDisparity(matrix);
  int numberOfElements = 0;
  float temp = 0;

  for (int r = 0; r < matrix.rows; ++r)
  {
    for (int c = 0; c < matrix.cols; ++c)
    {
      if (static_cast<float>(matrix.at<short>(r,c) > 0))
      {
        float value = static_cast<float>(matrix.at<float>(r,c));
        temp += pow(value - mean,2);
        ++numberOfElements;
      }
    }
  }

  float variance = temp / numberOfElements;
  return sqrt(variance);
}


void Utility::subdivideImage(cv::Mat const& subimage, int binning, std::vector<cv::Mat> &output)
{
  int width = subimage.cols;
  int height = subimage.rows;

  if (width == 0 || height == 0)
    LOG(INFO)<< mTag <<"Unable to subdivide Subimage. Input Mat Dimensions zero!\n";

  cv::Rect tmpRect;
  cv::Mat tmpMat;
  int x1,x2,y1,y2;

  // clear vector in order to release the matrices stored inside of it
  output.clear();

  for (int i = 0; i < 9; ++i)
  {
    if(i < 3)
    {
      x1 = i%3*(width/3);
      x2 = (i%3+1)*(width/3);
      y1 = 0;
      y2 = height/3;
      
      tmpRect = cv::Rect(cv::Point(x1,y1),cv::Point(x2,y2));
      // std::cout << tmpRect << std::endl;
      tmpMat = subimage(tmpRect);

      output.push_back(tmpMat);
    }
    else if(i > 2 && i < 6)
    {
      x1 = i%3*(width/3);
      x2 = (i%3+1)*(width/3);
      y1 = height/3;
      y2 = height/3*2;

      tmpRect = cv::Rect(cv::Point(x1,y1),cv::Point(x2,y2));
      // std::cout << tmpRect << std::endl;
      tmpMat = subimage(tmpRect);

      output.push_back(tmpMat);
    }
    else if (i > 5)
    {
      x1 = i%3*(width/3);
      x2 = (i%3+1)*(width/3);
      y1 = height/3*2;
      y2 = height;

      tmpRect = cv::Rect(cv::Point(x1,y1),cv::Point(x2,y2));
      // std::cout << tmpRect << std::endl;
      tmpMat = subimage(tmpRect);

      output.push_back(tmpMat);
    }
  }

  tmpMat.release();
}

std::string Utility::type2str(int type) {
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

float Utility::calcMagnitude(cv::Mat const& input)
{
  float mag = 0;
  std::for_each(input.begin<float>(), input.end<float>(), [&mag](float value) {
   mag += pow(value,2);
  });
  return mag;
}

float Utility::calcAngle(cv::Mat const& m1, cv::Mat const& m2)
{
  float angle = m1.dot(m2) / (calcMagnitude(m1)*calcMagnitude(m2));
  return acos(angle) * (180/ M_PI);
}