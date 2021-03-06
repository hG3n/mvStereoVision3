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

cv::Mat Utility::calcCoordinate(dMapValues dMapValues, cv::Mat const& Q)
{
  float d = dMapValues.dValue / 16;

  cv::Mat_<float> coordinate(1,4);

  coordinate(0)=dMapValues.image_x;
  coordinate(1)=dMapValues.image_y;
  coordinate(2)=d;
  coordinate(3)=1;

  coordinate = (Q)*coordinate.t();
  coordinate/=coordinate(3);

  float distance = coordinate(2)/1000;

  if(cvIsInf(distance))
  {
    coordinate(2) = 0;
    return coordinate;    
  } else {
    return coordinate;
  }

}

float Utility::calcDistance(dMapValues dMapValues, cv::Mat const& Q, int binning)
{
  float d = dMapValues.dValue / 16;
  cv::Mat_<float> coordinateQ(1,4);

  coordinateQ(0)=dMapValues.image_x;
  coordinateQ(1)=dMapValues.image_y;
  coordinateQ(2)=d;
  coordinateQ(3)=1;

  coordinateQ = (Q)*coordinateQ.t();
  coordinateQ/=coordinateQ(3);

  float distance = coordinateQ(2)/1000;
  coordinateQ.release();

  if(cvIsInf(distance))
    return 0;
  else
    return distance;
}

dMapValues Utility::calcDMapValues(cv::Mat_<float> const& c, cv::Mat const& Q)
{
  float numerator = Q.at<float>(2,3) - c(2) * Q.at<float>(3,3);
  float denominator = c(2) * Q.at<float>(3,2);
  float disparity_value = numerator/denominator;

  float image_x = c(0) * (disparity_value * Q.at<float>(3,2) * Q.at<float>(3,3)) + Q.at<float>(0,3);
  float image_y = c(1) * (disparity_value * Q.at<float>(3,2) * Q.at<float>(3,3)) + Q.at<float>(1,3);

  dMapValues toReturn;
  toReturn.image_x = image_x; 
  toReturn.image_y = image_y;
  // multiply by 16 in order to invert the normalization
  toReturn.dValue = disparity_value * 16;

  return toReturn;
}

void Utility::dmap2pcl(std::string const& filename, cv::Mat const& dMap, cv::Mat const& Q)
{
  dMapValues dMapValues;
  std::vector<cv::Mat> storage;
  ply p("Hagen Hiller", "disparity pointcloud", dMap);

  for( int r = 0; r < dMap.rows; ++r) {
    for(int c = 0; c < dMap.cols; ++c) {
      float value = dMap.at<short>(r,c);
      if(value > 0){
        dMapValues.image_x = c;
        dMapValues.image_y = r;
        dMapValues.dValue = value;

        cv::Mat coordinate = Utility::calcCoordinate(dMapValues, Q);
        storage.push_back(coordinate);   
      }
    }
  }
  p.write(filename, storage, ply::MODE::WITH_COLOR);
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
  } else {
    float mean = total / abs(numElements);
    return mean;
  }
}

std::pair<short,short> Utility::calcMinMaxDisparity(cv::Mat const& matrix)
{

  std::vector<short> elements;
  for (int r = 0; r < matrix.rows; ++r) {
    for (int c = 0; c < matrix.cols; ++c) {
      short value = matrix.at<short>(r,c);
      if(value > 0){
        elements.push_back(value);
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



std::string type2str(cv::Mat const& input) {
  std::string r;

  uchar depth = input.type() & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (input.type() >> CV_CN_SHIFT);

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
  std::for_each(input.begin<float>(), input.end<float>(), [&mag](float value){
    mag += pow(value,2);
  });

  return sqrt(mag);
}

float Utility::calcAngle(cv::Mat& m1, cv::Mat& m2)
{
  float mag1 = calcMagnitude(m1);
  float mag2 = calcMagnitude(m2);

  // if the length of one of the vectors cannot be calculated return 0
  // reason for that moght be an invalid disparity value
  if(mag1 == 0 || mag2 == 0) {
    return 0.0;
  }

  // if the matrix dimensions don't match transpose one of the matrices
  // in order to make the dotproduct work
  if(m1.cols != m2.cols) {
    m1 = m1.t();
  }

  std::cout << m1 << "  " << m2 << std::endl;

  float angle = m1.dot(m2) / (mag1 * mag2);
  return acos(angle) * (180/ M_PI);
}