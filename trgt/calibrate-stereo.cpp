#include <iostream>
#include <string>
#include <cstdio>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"

#include <thread>
#include <mutex>
#include <condition_variable>

INITIALIZE_EASYLOGGINGPP

int main(int argc, char const *argv[])
{

  std::string tag = "MAIN\t";

  LOG(INFO) << tag << "Application started.";
  mvIMPACT::acquire::DeviceManager devMgr;

  Camera *left;
  Camera *right;

  if(!Utility::initCameras(devMgr,left,right))
  {
    return 0;
  }

  Stereosystem stereo(left,right);

  std::vector<std::string> nodes;
  nodes.push_back("inputImages");
  nodes.push_back("outputParameter");

  std::string config = "./configs/default.yml";

  cv::FileStorage fs;


  if(!Utility::checkConfig(config,nodes,fs))
  {
    return 0;
  }


  std::string dirPath;
  fs["inputImages"] >> dirPath;

  std::string parameterOutput;
  fs["outputParameter"] >> parameterOutput;

  if(!Utility::createDirectory(parameterOutput))
  {
    LOG(ERROR) << tag << "Unable to create output directory for parameter" << std::endl;
  }

  std::vector<std::string> filenamesLeft;
  std::vector<std::string> filenamesRight;

  Utility::getFiles(dirPath+"/left", filenamesLeft);
  Utility::getFiles(dirPath+"/right", filenamesRight);

  std::vector<cv::Mat> imagesLeft, imagesRight;

  if(filenamesLeft.size() == filenamesRight.size()) {
    for(unsigned int i = 0; i < filenamesLeft.size(); ++i){
      imagesLeft.push_back(cv::imread(std::string(dirPath+"/left/"+filenamesLeft[i])));
      imagesRight.push_back(cv::imread(std::string(dirPath+"/right/"+filenamesRight[i])));
    }
  }

  double stereoRMS = stereo.calibrate(imagesLeft, imagesRight,20, cv::Size(11,8));
  // double stereoRMS = stereo.calibrate(imagesLeft, imagesRight,20, cv::Size(5,7));

  std::cout << "RMS after stereorectification: " <<  stereoRMS << std::endl;

  std::cout << "press 's' to save the new parameters." << std::endl;
  char key = getchar();
  if(key == 's')
  {
    stereo.saveIntrinsic(parameterOutput + "/intrinsic.yml");
    stereo.saveExtrinsic(parameterOutput + "/extrinsic.yml");
  }

  return 0;
}
