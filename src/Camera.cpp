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
	mBinningMode(0),
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
	mBinningMode(0)
{
	LOG(INFO)<< mTag <<"Camera created." << std::endl;
	this->setPixelFormat(MONO8);
	this->setExposure(12000);
	this->setGain(0);
	std::vector<char> v;
	this->getImage(v);
}

Camera::~Camera()
{
	LOG(INFO)<< mTag <<"Camera destroyed." << std::endl;
}


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
			imageToReturn= std::vector<char>(static_cast<char*>(mRequest->imageData.read()),
									                     static_cast<char*>(mRequest->imageData.read()) +\
															         mRequest->imageSize.read());
			mWidth = mRequest->imageWidth.read();
			mHeight = mRequest->imageHeight.read();
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
	std::vector<std::vector<cv::Point3f> > objectPoints;
  std::vector<std::vector<cv::Point2f> > imagePoints;
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

    bool found = cv::findChessboardCorners( grayImage, chessboardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if(found) {
      cv::cornerSubPix(grayImage, corners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.1));
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

  return rms;
}

void Camera::setExposureMode(unsigned int mode)
{
  switch(mode)
  {
    case AUTO_EXPOSURE_OFF:
      mCameraSettingsBlueFOX.autoExposeControl.write(mvIMPACT::acquire::TAutoExposureControl::aecOff);
      LOG(INFO) << mTag << "Set Auto Exposure Mode: OFF" << std::endl;
      break;
    case AUTO_EXPOSURE:
      mCameraSettingsBlueFOX.autoExposeControl.write(mvIMPACT::acquire::TAutoExposureControl::aecOn);
      LOG(INFO) << mTag << "Set Auto Exposure Mode: ON" << std::endl;
      break;
  }
}

void Camera::setExposure(unsigned int exposure)
{
	mCameraSettingsBlueFOX.expose_us.write(exposure);
	LOG(INFO) << mTag << "Set exposure time to " << exposure << std::endl;
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
		case MONO8:
			mImageDestinaton.pixelFormat.write(mvIMPACT::acquire::idpfMono8);
			mCameraSettingsBase.pixelFormat.write(mvIMPACT::acquire::ibpfMono8);
			LOG(INFO) << mTag << "Set Pixelformat to Mono8" << std::endl;
			break;
		default:
			LOG(WARNING) << mTag << "Unknown Pixelformat" <<std::endl;
		//doesnt work
		/*case MONO16:
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
		case BINNING_OFF:
			mCameraSettingsBlueFOX.binningMode.write(mvIMPACT::acquire::cbmOff);
			LOG(INFO) << mTag << "Set binning mode off." <<std::endl;
			mBinningMode = BINNING_OFF;
			break;
		case BINNING_HV:
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
