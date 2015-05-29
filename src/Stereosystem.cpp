#include "Stereosystem.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

Stereosystem::Stereosystem(Camera* l, Camera* r):
  mLeft(l),
  mRight(r),
  mR(),
  mT(),
  mE(),
  mF(),
  mIsInit(false),
  mIntrinsicLeft(),
  mIntrinsicRight(),
  mDistCoeffsLeft(),
  mDistCoeffsRight(),
  mTag("STEREOSYSTEM\t")
{
  LOG(INFO)<< mTag <<"Stereosystem created\n";
}

Stereosystem::~Stereosystem()
{
  mR.release();
  mT.release();
  mE.release();
  mF.release();

  mMap1[0].release();
  mMap2[1].release();
  mR0.release();
  mR1.release();
  mP0.release();
  mP1.release();
  mQ.release();
  mIntrinsicLeft.release();
  mIntrinsicRight.release();
  mDistCoeffsLeft.release();
  mDistCoeffsRight.release();

  LOG(INFO)<< mTag <<"Stereosystem destroyed\n";
}

double Stereosystem::calibrate(std::vector<cv::Mat> const& leftImages,
                               std::vector<cv::Mat> const& rightImages,
                               double patternSize, cv::Size chessboardSize)

{

  if(leftImages.size() == rightImages.size())
  {
    // calibrate each camera at first to get intrinsics
    mLeft->calibrate(leftImages,patternSize, chessboardSize);
    mRight->calibrate(rightImages,patternSize, chessboardSize);

    mIntrinsicLeft = mLeft->getIntrinsic();
    mIntrinsicRight = mRight->getIntrinsic();
    mDistCoeffsLeft = mLeft->getDistCoeffs();
    mDistCoeffsRight = mRight->getDistCoeffs();

    // needed calibration variables
    std::vector<std::vector<cv::Point3f> > objectPoints;
    std::vector<std::vector<cv::Point2f> > imagePointsLeft;
    std::vector<std::vector<cv::Point2f> > imagePointsRight;
    std::vector<cv::Point2f> cornersLeft;
    std::vector<cv::Point2f> cornersRight;
    std::vector<cv::Point3f> obj;

    // size of calibration patteren squares
    for(int y=0; y<chessboardSize.height; ++y) {
      for(int x=0; x<chessboardSize.width; ++x) {
        obj.push_back(cv::Point3f((float(x)*patternSize),(float(y)*patternSize),0));
      }
    }

    for(unsigned int i = 0; i < leftImages.size(); ++i) {
      cv::Mat grayImageLeft;
      cv::Mat grayImageRight;
      cv::cvtColor(leftImages[i], grayImageLeft, CV_BGR2GRAY);
      cv::cvtColor(rightImages[i], grayImageRight, CV_BGR2GRAY);

      bool foundL = cv::findChessboardCorners( grayImageLeft, chessboardSize, cornersLeft, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
      bool foundR = cv::findChessboardCorners( grayImageRight, chessboardSize, cornersRight, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

      if(foundL && foundR) {
        cv::cornerSubPix(grayImageLeft, cornersLeft, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.1));
        cv::cornerSubPix(grayImageRight, cornersRight, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.1));

        imagePointsLeft.push_back(cornersLeft);
        imagePointsRight.push_back(cornersRight);

        objectPoints.push_back(obj);

        LOG(INFO) << "Found chessboard in image: " << i << std::endl;

        grayImageLeft.release();
        grayImageRight.release();
      }
      else
      {
        LOG(INFO) << "Unable to find Corners in image " << i << ". Image ignored" << std::endl;
        continue;
      }
    }

    // calibrate the stereo system
    cv::Size imagesize = cv::Size(leftImages[0].size());
    double stereoRMS = cv::stereoCalibrate(objectPoints,imagePointsLeft,imagePointsRight,
                                           mIntrinsicLeft,mDistCoeffsLeft,mIntrinsicRight,mDistCoeffsRight,
                                           imagesize,mR,mT,mE,mF,
                                           cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 300, 1e-7));

    return stereoRMS;
  }

  return -1.0;
}

bool Stereosystem::loadExtrinisic(std::string const& file )
{
  cv::FileStorage fs;
  bool success = fs.open(file, cv::FileStorage::READ);

  if(fs["R"].empty() || fs["T"].empty() || fs["E"].empty() || fs["F"].empty())
  {
    LOG(ERROR) << mTag << "Node in " << file << "is empty." <<std::endl;
      fs.release();
    return false;
  }

  if(success)
  {
      fs["R"] >> mR;
      fs["T"] >> mT;
      fs["E"] >> mE;
      fs["F"] >> mF;
      LOG(INFO) << mTag <<"Successfully loaded Extrinsics." << std::endl;
      fs.release();
      return true;
  }
  else
  {
    LOG(ERROR) << mTag << "Unable to open extrinsic file: " << file <<std::endl;
      fs.release();
    return false;
  }
}

bool Stereosystem::loadIntrinsic(std::string const& file)
{
  cv::FileStorage fs;
  bool success = fs.open(file, cv::FileStorage::READ);

  if(fs["cameraMatrixLeft"].empty() || fs["cameraMatrixRight"].empty() || fs["distCoeffsRight"].empty() || fs["distCoeffsRight"].empty())
  {
    LOG(ERROR) << mTag << "Node in " << file << "is empty." << std::endl;
    fs.release();
    return false;
  }

  if(success)
    {
      fs["cameraMatrixLeft"] >> mIntrinsicLeft;
      fs["cameraMatrixRight"] >> mIntrinsicRight;
      fs["distCoeffsLeft"] >> mDistCoeffsLeft;
      fs["distCoeffsRight"] >> mDistCoeffsRight;
      mLeft->setIntrinsic(mIntrinsicLeft);
      mRight->setIntrinsic(mIntrinsicRight);
      LOG(INFO) << mTag <<"Successfully loaded Intrinsics." << std::endl;;
      fs.release();
      return true;
   }
  else
  {
    LOG(ERROR) << mTag << "Unable to open intrinsic file: " << file << std::endl;
      fs.release();
    return false;
  }
}

bool Stereosystem::saveExtrinsic(std::string const& file)
{
  cv::FileStorage fs;
  bool success = fs.open(file, cv::FileStorage::WRITE);
  if(success)
  {
    fs << "R" << mR;
      fs << "T" << mT;
      fs << "E" << mE;
      fs << "F" << mF;

    // if(fs["R"].empty() || fs["T"].empty() || fs["E"].empty() || fs["F"].empty())
    // {
   //     LOG(ERROR) << mTag << "Unable to save extrinsic to " << file << ". Empty Node" << std::endl;
   //     fs.release();
    //  return false;
    // }
      LOG(INFO) << mTag <<"Successfully saved Extrinsics to " << file << std::endl;;
    fs.release();
    return true;
  }
    else
    {
      LOG(ERROR) << mTag <<"Unable to open " << file << " for saving." << std::endl;;
      fs.release();
    return false;
    }
}

bool Stereosystem::saveIntrinsic(std::string const& file)
{
  cv::FileStorage fs;
  bool success = fs.open(file, cv::FileStorage::WRITE);
  if(success)
  {
    fs << "cameraMatrixLeft" << mIntrinsicLeft;
    fs << "cameraMatrixRight" << mIntrinsicRight;
    fs << "distCoeffsLeft" << mDistCoeffsLeft;
    fs << "distCoeffsRight" << mDistCoeffsRight;

    // if(fs["cameraMatrixLeft"].empty() || fs["cameraMatrixRight"].empty() || fs["distCoeffsLeft"].empty() || fs["distCoeffsRight"].empty())
   //   {
   //     LOG(ERROR) << mTag << "Unable to save intrisic to " << file << ". Empty Node" << std::endl;
   //     fs.release();
    //  return false;
   //   }
      LOG(INFO) << mTag <<"Successfully saved Intrinsics to " << file << std::endl;
      fs.release();
    return true;
  }
  else
  {
    LOG(ERROR) << mTag <<"Unable to open " << file << " for saving." << std::endl;;
      fs.release();
    return false;
  }

}

bool Stereosystem::getImagepair(Stereopair& stereoimagepair)
{
  std::vector<char> leftImage;
  //mLeft->getImage(leftImage);
  std::vector<char> rightImage;
  //mRight->getImage(rightImage);
  //std::future<bool> l = std::async(std::launch::async,&Camera::getImage,mLeft,std::ref(leftImage));
  //std::future<bool> r = std::async(std::launch::async,&Camera::getImage,mRight,std::ref(rightImage));

  std::thread t1(&Camera::getImage,mLeft,std::ref(leftImage));
  std::thread t2(&Camera::getImage,mRight,std::ref(rightImage));

  t1.join();
  t2.join();
  cv::Mat(mLeft->getImageHeight(),mLeft->getImageWidth(), CV_8UC1, &leftImage[0]).copyTo(stereoimagepair.mLeft);
  cv::Mat(mRight->getImageHeight(),mRight->getImageWidth(), CV_8UC1, &rightImage[0]).copyTo(stereoimagepair.mRight);
  return true;
}

bool Stereosystem::getUndistortedImagepair(Stereopair& sip)
{
  cv::Mat tmpLeft, tmpRight;
  if(this->getImagepair(sip))
  {
    cv::undistort(sip.mLeft, tmpLeft, mIntrinsicLeft, mDistCoeffsLeft);
    cv::undistort(sip.mRight, tmpRight, mIntrinsicRight, mDistCoeffsRight);
    tmpLeft.copyTo(sip.mLeft);
    tmpRight.copyTo(sip.mRight);
    return true;
  }
  return false;
}

bool Stereosystem::initRectification()
{
  LOG(INFO) << mTag << "Called initRectification()" << std::endl;

  cv::Size imagesizeL(mLeft->getImageWidth(), mLeft->getImageHeight());
  cv::Size imagesizeR(mRight->getImageWidth(), mRight->getImageHeight());
  LOG(INFO) << mTag << "Imagessizes are: "<< imagesizeL << " "<< imagesizeR <<std::endl;

  if(imagesizeL == imagesizeR)
  {
    if(mLeft->getBinningMode() && mRight->getBinningMode())
    {
      mIntrinsicLeft /=2;
      mIntrinsicRight/=2;
    }

    // initialize all Rectification parameters
    cv::stereoRectify(mIntrinsicLeft, mDistCoeffsLeft, mIntrinsicRight, mDistCoeffsRight,
                      imagesizeL, mR, mT, mR0, mR1, mP0, mP1, mQ, CV_CALIB_ZERO_DISPARITY, 0,
                      imagesizeL, &mValidROI[0], &mValidROI[1]);

    cv::initUndistortRectifyMap(mIntrinsicLeft, mDistCoeffsLeft, mR0, mP0,
                                imagesizeL, CV_32FC1, mMap1[0], mMap2[0]);
    cv::initUndistortRectifyMap(mIntrinsicRight, mDistCoeffsRight, mR1, mP1,
                                imagesizeL, CV_32FC1, mMap1[1], mMap2[1]);


    // just get the image contained by both ROIs
    mDisplayROI = mValidROI[0] & mValidROI[1];

    LOG(INFO) << mTag << "Q Matrix: " << mQ << std::endl;
    LOG(INFO) << mTag << "ROI 1: " << mValidROI[0] << " | ROI 2: " << mValidROI[1] << std::endl;
    LOG(INFO) << mTag << "Current Display ROI: " << mDisplayROI << std::endl;

    LOG(INFO) << mTag << "Rectification successfully initialized! "<< mDisplayROI <<std::endl;
    mIsInit = true;

    if(mLeft->getBinningMode() && mRight->getBinningMode())
    {
      mIntrinsicLeft *=2;
      mIntrinsicRight*=2;
    }
      return true;
  }
  else
  {
    LOG(ERROR) << mTag << "Unable to init rectification" <<std::endl;
    return false;
  }
  return false;
}

bool Stereosystem::getRectifiedImagepair(Stereopair& sip)
{

  if(!this->getImagepair(sip))
  {
    return false;
  }
  if(mIsInit)
  {
    cv::remap(sip.mLeft, sip.mLeft, mMap1[0], mMap2[0], cv::INTER_LINEAR);
    cv::remap(sip.mRight, sip.mRight, mMap1[1], mMap2[1], cv::INTER_LINEAR);

    sip.mLeft = sip.mLeft(mDisplayROI);
    sip.mRight = sip.mRight(mDisplayROI);
    return true;
  }
  else
  {
    if(!this->initRectification())
    {
      return false;
    }
    else
    {
      cv::remap(sip.mLeft, sip.mLeft, mMap1[0], mMap2[0], cv::INTER_LINEAR);
      cv::remap(sip.mRight, sip.mRight, mMap1[1], mMap2[1], cv::INTER_LINEAR);

      sip.mLeft = sip.mLeft(mDisplayROI);
      sip.mRight = sip.mRight(mDisplayROI);
      return true;
    }
    return false;
  }
  return false;
 }

 void Stereosystem::resetRectification()
 {
  mIsInit = false;
 }

 void Stereosystem::getFundamentalMatrix(cv::Mat &fundamental) const
 {
  mF.copyTo(fundamental);
 }

 void Stereosystem::getTranslationMatrix(cv::Mat &translation) const
 {
  mT.copyTo(translation);
 }

 double Stereosystem::getBaseline() const
 {
  return sqrt(pow(mT.at<double>(0,0),2) + pow(mT.at<double>(1,0),2) + pow(mT.at<double>(2,0),2));
  // return mT.at<double>(0,0);
 }

 cv::Mat Stereosystem::getRotationMatrix() const
 {
  return mR;
 }

cv::Mat Stereosystem::getQMatrix() const
{
  return mQ;
}

// function to return 'after rectification camera matrices' P0/P1
std::vector<cv::Mat> Stereosystem::getNewCameraMatrices() const
{
  std::vector<cv::Mat> v;
  v.push_back(mP0);
  v.push_back(mP1);
  return v;
}