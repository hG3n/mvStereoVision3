#include "Camera.h"

Camera::Camera():
	mDevice(NULL),
	mFunctionInterface(NULL),
	mStatistics(NULL),
  mSystemSettings(NULL),
  mCameraSettingsBase(NULL),
  mCameraSettingsBlueFOX(NULL),
	mImageDestinaton(NULL),
	mTimeout(1000),
	mTag(" CAMERA "),
	mWidth(0),
	mHeight(0),
	mBinningMode(),
	mIntrinsic(),
	mDistCoeffs()
	{}

Camera::Camera(mvIMPACT::acquire::Device* dev):
	mDevice(dev),
	mFunctionInterface(dev),
	mStatistics(dev),
  mSystemSettings(dev),
  mCameraSettingsBase(dev),
  mCameraSettingsBlueFOX(dev),
  mImageDestinaton(dev),
	mTimeout(1000),
	mTag("CAMERA\tSerial:"+mDevice->serial.read() +\
		 " ID:" + std::to_string(mDevice->deviceID.read())+ "\t"),
	mWidth(0),
	mHeight(0),
	mBinningMode(binning::BINNING_OFF)
{
  LOG(INFO)<< mTag <<"Camera created." << std::endl;
  this->setPixelFormat(pixelformat::MONO8);
  this->setExposure(12000);
  this->setGain(0);
  this->enableAutoExposure(exposure::DISABLE_AUTO_EXP);
  this->enableAutoGain(gain::DISABLE_GAIN);
  std::vector<char> v;
  this->getImage(v);
}

Camera::~Camera()
{
	LOG(INFO)<< mTag <<"Camera destroyed." << std::endl;
}

// -----------------------------------------------------------------------------
// --- getter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
float Camera::getFramerate() const
{
  return mStatistics.framesPerSecond.read();
}

unsigned int Camera::getImageWidth() const
{
  return mWidth;
}

unsigned int Camera::getImageHeight() const
{
  return mHeight;
}

int Camera::getExposure() const
{
  return mCameraSettingsBlueFOX.expose_us.read();
}

float Camera::getGain() const
{
  return mCameraSettingsBlueFOX.gain_dB.read();
}

int Camera::getBinningMode() const
{
  return mBinningMode;
}

cv::Mat Camera::getIntrinsic() const
{
  return mIntrinsic;
}

cv::Mat Camera::getDistCoeffs() const
{
  return mDistCoeffs;
}

// -----------------------------------------------------------------------------
// --- setter ------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Camera::enableAutoExposure(unsigned int mode)
{
  switch(mode)
  {
    case exposure::ENABLE_AUTO_EXP:
      mCameraSettingsBlueFOX.autoExposeControl.write(mvIMPACT::acquire::TAutoExposureControl::aecOn);
      LOG(INFO) << mTag << "Set Auto Exposure Mode: ON" << std::endl;
      break;
    case exposure::DISABLE_AUTO_EXP:
      mCameraSettingsBlueFOX.autoExposeControl.write(mvIMPACT::acquire::TAutoExposureControl::aecOff);
      LOG(INFO) << mTag << "Set Auto Exposure Mode: OFF" << std::endl;
      break;
  }
}

void Camera::setExposure(unsigned int exposure)
{
  mCameraSettingsBlueFOX.expose_us.write(exposure);
  LOG(INFO) << mTag << "Set exposure time to " << exposure << std::endl;
}

void Camera::enableAutoGain(unsigned int option)
{
  switch(option)
  {
    case gain::ENABLE_GAIN:
      mCameraSettingsBlueFOX.autoGainControl.write(mvIMPACT::acquire::TAutoGainControl::agcOn);
      LOG(INFO) << mTag << "enabled Auto Gain Control!" << std::endl;
      break;
    case gain::DISABLE_GAIN:
      mCameraSettingsBlueFOX.autoGainControl.write(mvIMPACT::acquire::TAutoGainControl::agcOff);
      LOG(INFO) << mTag << "disabled Auto Gain Control" << std::endl;
      break;
  }
}

void Camera::setGain(float gain)
{
  mCameraSettingsBlueFOX.gain_dB.write(gain);
  LOG(INFO) << mTag << "Set gain DB to " << gain << std::endl;
}

void Camera::setPixelFormat(int option)
{
  switch(option)
  {
    case pixelformat::MONO8:
      mImageDestinaton.pixelFormat.write(mvIMPACT::acquire::idpfMono8);
      mCameraSettingsBase.pixelFormat.write(mvIMPACT::acquire::ibpfMono8);
      LOG(INFO) << mTag << "Set Pixelformat to Mono8" << std::endl;
      break;
    default:
      LOG(WARNING) << mTag << "Unknown Pixelformat" <<std::endl;
    //doesnt work
    /*case pixelformat::MONO16:
      mImageDestinaton.pixelFormat.write(mvIMPACT::acquire::idpfMono16);
      mCameraSettingsBase.pixelFormat.write(mvIMPACT::acquire::ibpfMono16);
      LOG(INFO) << mTag << "Set Pixelformat to Mono16" << std::endl;
      break;*/
  }
}

void Camera::setBinning(unsigned int option)
{
  switch(option)
  {
    case binning::BINNING_OFF:
      mCameraSettingsBlueFOX.binningMode.write(mvIMPACT::acquire::cbmOff);
      LOG(INFO) << mTag << "Set binning mode off." <<std::endl;
      mBinningMode = BINNING_OFF;
      break;
    case binning::BINNING_HV:
      mCameraSettingsBlueFOX.binningMode.write(mvIMPACT::acquire::cbmBinningHV);
      LOG(INFO) << mTag << "Set horizontal and vertical binning mode." <<std::endl;
      mBinningMode = BINNING_HV;
      break;
    default:
      LOG(WARNING)<< mTag << "Unknown binning mode: " << option <<\
                 ". No binning performed." << std::endl;
      break;
  }
}

void Camera::setIntrinsic(cv::Mat intrinsic)
{
  intrinsic.copyTo(mIntrinsic);
}

void Camera::setRequestTimeout(int timeout)
{
  mTimeout = timeout;
}

void Camera::setPixelClock(int clock)
{
  mCameraSettingsBlueFOX.pixelClock_KHz.write(mvIMPACT::acquire::cpc40000KHz);
}

void Camera::enableHDR(int option)
{
  switch(option)
  {
    case hdr::ENABLE_HDR:
      mCameraSettingsBlueFOX.getHDRControl().HDREnable.write(mvIMPACT::acquire::TBoolean::bTrue);
      LOG(INFO) << mTag << "HDR enabled!" << std::endl;
      break;
    case hdr::DISABLE_HDR:
      mCameraSettingsBlueFOX.getHDRControl().HDREnable.write(mvIMPACT::acquire::TBoolean::bFalse);
      LOG(INFO) << mTag << "HDR disabled!" << std::endl;
      break;
  }
}

// -----------------------------------------------------------------------------
// --- functions ---------------------------------------------------------------
// -----------------------------------------------------------------------------
bool Camera::getImage(std::vector<char> &imageToReturn)
{

  int result = DMR_NO_ERROR;
    //request an image
    result = mFunctionInterface.imageRequestSingle();

    if(result != DMR_NO_ERROR)
    {
      std::cerr << "Error while requesting for image: "<<\
      mvIMPACT::acquire::ImpactAcquireException::getErrorCodeAsString(result)<<\
      std::endl;
      LOG(ERROR)<< mTag << "Error while requesting for image: "<<\
      mvIMPACT::acquire::ImpactAcquireException::getErrorCodeAsString(result)<<\
      std::endl;
      return false;
    }

    int requestNr = mFunctionInterface.imageRequestWaitFor(mTimeout);
    if(mFunctionInterface.isRequestNrValid(requestNr))
    {
      mRequest = mFunctionInterface.getRequest(requestNr);

      if(mRequest->isOK())
      {
        //create vector with image data from request

        mWidth = mRequest->imageWidth.read();
        mHeight = mRequest->imageHeight.read();

        // std::memcpy(mat.ptr(),static_cast<char*>(mRequest->imageData.read()), mRequest->imageSize.read());
        // mat = cv::Mat(mHeight, mWidth, CV_8UC1, static_cast<char*>(mRequest->imageData.read()));
        imageToReturn= std::vector<char>(static_cast<char*>(mRequest->imageData.read()),
                                         static_cast<char*>(mRequest->imageData.read()) +\
                                         mRequest->imageSize.read());

        mFunctionInterface.imageRequestUnlock(requestNr);
        return true;
      }
      else
      {
        std::cerr << "Error, request not OK!" <<std::endl;
        LOG(ERROR) << mTag << "Error, request not OK!" <<std::endl;
        return false;
      }
    }
    LOG(ERROR) << mTag << "Error, invalid requestnumber!" << std::endl;
    mFunctionInterface.imageRequestUnlock(requestNr);

    return false;
}

double Camera::calibrate(std::vector<cv::Mat> const& images, double patternsize, cv::Size chessboardSize)
{
  // needed calibration variables
	std::vector<cv::Mat> rvecs,tvecs;
	std::vector<std::vector<cv::Point3f>> objectPoints;
  std::vector<std::vector<cv::Point2f>> imagePoints;
  std::vector<cv::Point2f> corners;
  std::vector<cv::Point3f> obj;

  // camera matrices
  cv::Mat intrinsic, distCoeffs;

  // size of calibration patteren squares
  for(int y=0; y<chessboardSize.height; ++y) {
    for(int x=0; x<chessboardSize.width; ++x) {
      obj.push_back(cv::Point3f((float(x)*patternsize),(float(y)*patternsize),0));
    }
  }

	for(unsigned int i = 0; i < images.size(); ++i) {
		cv::Mat grayImage;
  	cv::cvtColor(images[i], grayImage, CV_BGR2GRAY);

    // try to find chessboard corners within an image
    bool found = cv::findChessboardCorners( grayImage, chessboardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

		if(found) {
      cv::cornerSubPix(grayImage, corners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
      imagePoints.push_back(corners);
      objectPoints.push_back(obj);
      std::cout << "Found " << i << std::endl;
      grayImage.release();
		}
		else
		{
			std::cout << "Unable to find Corners in image " << i << ". Image ignored" << std::endl;
			continue;
		}
  }

  // calibrate the camera
  cv::Size imagesize = cv::Size(images[0].size());
  double rms = cv::calibrateCamera(objectPoints, imagePoints, imagesize, intrinsic, distCoeffs, rvecs, tvecs);

  // assign intrinsic and extrinsic to camera
  mIntrinsic = intrinsic;
  mDistCoeffs = distCoeffs;

  // return reprojection error to determine the quality of a calibration
  return rms;
}
