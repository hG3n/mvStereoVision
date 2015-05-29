#include <iostream>
#include <string>

//system command
#include <cstdlib>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"

#include <thread>
#include <mutex>
#include <condition_variable>

INITIALIZE_EASYLOGGINGPP

int main(int argc, char* argv[])
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

    Stereopair s;

    std::vector<std::string> nodes;
    nodes.push_back("outputImages");

    std::string config = "./configs/default.yml";

    cv::FileStorage fs;


    if(!Utility::checkConfig(config,nodes,fs))
    {
        return 0;
    }


    std::string dirPath;
    fs["outputImages"] >> dirPath;

    std::string pathLeft = dirPath+"/left";
    std::string pathRight = dirPath+"/right";


    int imageNumber = 0;


    if(Utility::directoryExist(pathLeft) || Utility::directoryExist(pathRight))
  {
    std::vector<std::string> tmp1, tmp2;
    Utility::getFiles(pathLeft,tmp1);
    Utility::getFiles(pathRight,tmp2);

    if(tmp1.size() != 0 || tmp2.size() != 0)
    {
      std::cout << "Output directory not empty, clean files? [y/n] " <<std::endl;
      char key = getchar();
        if(key == 'y')
        {
          std::system(std::string("rm " + pathLeft + "/* -f ").c_str());
          std::system(std::string("rm " + pathRight + "/* -f ").c_str());
        }
        else if(key == 'n')
        {
          imageNumber = tmp1.size();
          LOG(INFO) << tag << "Start with image number " << imageNumber << std::endl;
        }
    }
  }

    if(Utility::createDirectory(pathLeft) && Utility::createDirectory(pathRight))
    {
            LOG(INFO) << tag << "Successfully created directories for captured images." << std::endl;
    }
    else
    {
        LOG(ERROR) << tag << "Unable to create directories for captured images." <<std::endl;
        return 0;
    }

    left->setExposureMode(1);
    right->setExposureMode(1);

    char key = 0;
    int binning = 0;
    cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
    bool running = true;
    std::string filename = "";
    while(running)
    {

        if(!stereo.getImagepair(s))
        {
            LOG(ERROR) << tag << "Unable to get imagepair" << std::endl;
            break;
        }
        cv::imshow("Left", s.mLeft);
        cv::imshow("Right", s.mRight);

        key = cv::waitKey(5);

        if(key > 0)
        {
            switch(key)
            {
                case 'q':
                    running = false;
                    LOG(INFO) << tag << "Exit requested" <<std::endl;
                    delete left;
                    left = NULL;

                    delete right;
                    right = NULL;
                    break;
                case 'b':
                    if (binning == 0)
                        binning = 1;
                    else
                        binning =0;

                    left->setBinning(binning);
                    right->setBinning(binning);
                    stereo.resetRectification();
                    break;
                case 'c':
                    filename = "";
                    if(imageNumber < 10)
                    {
                        filename+= "00" + std::to_string(imageNumber);
                    }
                    if(imageNumber >= 10 && imageNumber < 100)
                    {
                        filename+="0" + std::to_string(imageNumber);
                    }

                    cv::imwrite(pathLeft+"/left_"+filename+".jpg",s.mLeft);
                    cv::imwrite(pathRight+"/right_"+filename+".jpg",s.mRight);
                    LOG(INFO) << tag << "Wrote left image to " << std::string(pathLeft+"/left_"+filename+".jpg") <<std::endl;
                    LOG(INFO) << tag << "Wrote right image to " << std::string(pathRight+"/right_"+filename+".jpg") <<std::endl;
                    ++imageNumber;
                    break;
                case 'f':
                    std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;
                    break;
                default:
                    std::cout << "Key pressed has no action" <<std::endl;
                    break;
            }
        }
    }


    return 0;
}
