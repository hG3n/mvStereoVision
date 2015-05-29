#include <iostream>
#include <string>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"
#include "subimage.h"
#include "obstacleDetection.h"
#include "view.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#define MEAN_DISTANCE 0
#define MIN_DISTANCE 1
#define STDDEV 2
#define SAMPLE 3

INITIALIZE_EASYLOGGINGPP
bool running = true;

// threading stuff
std::mutex disparityLockSGBM;
std::condition_variable cond_var;

cv::StereoSGBM disparitySGBM;
int numDispSGBM = 64;
int windSizeSGBM = 9;
bool newDisparityMap = false;

cv::Mat dMapRaw;
cv::Mat dMapNorm;

cv::Mat Q, Q_32F;
cv::Mat R, R_32F;
cv::Mat disparityMap_32FC1;

void disparityCalc(Stereopair const& s, cv::StereoSGBM &disparity)
{
  while(running)
  {
    std::unique_lock<std::mutex> ul(disparityLockSGBM);
    cond_var.wait(ul);
    Disparity::sgbm(s, dMapRaw, disparity);
    dMapRaw.convertTo(disparityMap_32FC1,CV_32FC1);
    newDisparityMap=true;
  }
}

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


void mouseClick(int event, int x, int y,int flags, void* userdata)
{
  if  ( event == CV_EVENT_LBUTTONDOWN )
  {
    double d = static_cast<float>(dMapRaw.at<short>(y,x));
    float distance = Utility::calcDistance(Q_32F, d, 1);
    std::cout << "disparityValue: " << d << "  distance: " << distance << std::endl;
  }
}


void initWindows()
{
  cv::namedWindow("SGBM" ,1);
  cv::createTrackbar("Num Disp", "SGBM", &numDispSGBM, 320, changeNumDispSGBM);
  cv::createTrackbar("Wind Size", "SGBM", &windSizeSGBM, 51, changeWindSizeSGBM);
  cv::setMouseCallback("SGBM", mouseClick, NULL);
}


int main(int argc, char* argv[])
{
  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started." << std::endl;
  mvIMPACT::acquire::DeviceManager devMgr;

  Camera *left;
  Camera *right;

  if(!Utility::initCameras(devMgr,left,right))
  {
    return 0;
  }

  Stereosystem stereo(left,right);

  if(!stereo.loadIntrinsic("parameters/intrinsic.yml"))
  {
    return 0;
  }
  if(!stereo.loadExtrinisic("parameters/extrinsic.yml"))
  {
    return 0;
  }

  Stereopair s;
  left->setExposureMode(1);
  right->setExposureMode(1);

  char key = 0;
  int binning = 0;

  stereo.getRectifiedImagepair(s);
  Q = stereo.getQMatrix();
  Q.convertTo(Q_32F,CV_32F);

  cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
  initWindows();

  disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
  std::thread disparity(disparityCalc,std::ref(s),std::ref(disparitySGBM));

  obstacleDetection obst(dMapRaw, binning);

  running = true;
  int frame = 0;
  
  while(running)
  {

    stereo.getRectifiedImagepair(s);
    cv::imshow("Left", s.mLeft);
    cv::imshow("Right", s.mRight);

    // mean map storage
    std::vector<std::vector<float>> v;

    if(newDisparityMap)
    {
      obst.buildMeanDistanceMap(Q_32F);
      // obst.buildMinDistanceMap(Q_32F);
      // obst.buildStdDevDistanceMap(Q_32F);
      v = obst.getDistanceMapMean();
      // v = obst.getDistanceMapMin();
      // v = obst.getDistanceMapStdDev();
      if (binning == 0)
      {
        obst.detectObstacles(MEAN_DISTANCE, std::make_pair(1.2,2.0));
        // obst.detectObstacles(MIN_DISTANCE, std::make_pair(1.2,2.0));
      }
      else
      {
        obst.detectObstacles(MEAN_DISTANCE, std::make_pair(0.8,1.2));
        // obst.detectObstacles(MIN_DISTANCE, std::make_pair(0.8,1.2));
      }

      // display stuff
      cv::normalize(dMapRaw,dMapNorm,0,255,cv::NORM_MINMAX, CV_8U);
      cv::cvtColor(dMapNorm,dMapNorm,CV_GRAY2BGR);
      View::drawObstacleGrid(dMapNorm, binning);
      View::drawSubimageGrid(dMapNorm, binning);
      cv::imshow("SGBM",dMapNorm);
      newDisparityMap = false;
    }

    // notify the thread to start 
    cond_var.notify_one();
    key = cv::waitKey(5);

    // keypress stuff
    if(key > 0)
    {
      switch(key)
      {
        case 'q':
          LOG(INFO) << tag << "Exit requested" <<std::endl;
          delete left;
          left = nullptr;
          delete right;
          right = nullptr;
          cond_var.notify_one();
          running = false;
          break;
        case 'b':
          if (binning == 0)
            binning = 1;
          else
            binning = 0;
          left->setBinning(binning);
          right->setBinning(binning);
          stereo.resetRectification();
          break;
        case 'f':
          std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;
          break;
        case '0':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[0][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '1':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[1][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '2':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[2][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '3':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[3][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '4':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[4][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '5':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[5][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '6':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[6][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '7':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[7][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case '8':
          for (int i = 0; i < 9; ++i)
          {
            std::cout << i << ": " << v[8][i] << std::endl;
          }
          std::cout << "" << std::endl;
          break;
        case 'k':
          {
            cv::FileStorage fs2("newCameraMatrices.yml", cv::FileStorage::WRITE);
            std::vector<cv::Mat> u(stereo.getNewCameraMatrices());
            fs2 << "cameraMatrixNewLeft" << u[0];
            fs2 << "cameraMatrixNewRight" << u[1];
            fs2.release();
            std::cout << "new Camera Parameters saved successfully!" << std::endl;
          }
          break;
        default:
          std::cout << "Key pressed has no action" <<std::endl;
          break;
      }
    }
    ++frame;
  }

  disparity.join();

  return 0;
}
