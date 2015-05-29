#include <iostream>
#include <string>

#include "Stereosystem.h"
#include "disparity.h"
#include "easylogging++.h"

#include <thread>
#include <mutex>
#include <condition_variable>

INITIALIZE_EASYLOGGINGPP

//Thread stuff
std::mutex disparityLockSGBM,disparityLockBM, disparityLockTM;
std::condition_variable cond_var;
bool newDisparityMap = false;
bool running = true;
cv::Mat dispMapSGBM;

cv::StereoSGBM disparitySGBM;

int numDispSGBM = 16;
int windSizeSGBM = 5;

int numDispBM = 16;

double baseline ;
double fx, fy, cx, cy;

cv::Mat R, R_32F;
cv::Mat disparityMap_32FC1;

void disparityCalc(Stereopair const& s, cv::StereoSGBM &disparity)
{
	while(running)
	{
		std::unique_lock<std::mutex> ul(disparityLockSGBM);
		cond_var.wait(ul);
		Disparity::sgbm(s, dispMapSGBM, disparity);
        dispMapSGBM.convertTo(disparityMap_32FC1,CV_32FC1);

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

            double d = double(dispMapSGBM.at<unsigned char>(x,y));
            std::cout<<d<<std::endl;
            if(d > 0)
            {
                cv::Mat_<float>  p(3,1), pFinish(3,1);
                p(2) = baseline*fx/d;
                p(0) = p(2)*(x-cx)/fx;
                p(1) = p(2)*(y-cy)/fy;
                pFinish = R_32F.t()*p;
                std::cout<< p(0) << " " << p(1) << " " << p(2) << std::endl;
                std::cout<< pFinish(0) << " " << pFinish(1) << " " << pFinish(2) << std::endl;
                std::cout<<"distance: " << (pFinish(2))/1000.0 <<std::endl;
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


void initWindows()
{
	cv::namedWindow("SGBM" ,1);
    cv::namedWindow("BM" ,1);

	cv::createTrackbar("Num Disp", "SGBM", &numDispSGBM, 320, changeNumDispSGBM);
    cv::createTrackbar("Wind Size", "SGBM", &windSizeSGBM, 51, changeWindSizeSGBM);

    cv::createTrackbar("Num Disp", "BM", &numDispBM, 320, changeNumDispBM);
    cv::createTrackbar("Wind Size", "BM", &windSizeBM, 51, changeWindSizeBM);

    cv::setMouseCallback("SGBM", mouseClick, NULL);

}

// void disparityCalcNCC( Stereopair const& s) {
// 	while(running)
// 	{
// 		std::unique_lock<std::mutex> ul(disparityLock);
// 		cond_var.wait(ul);
// 		Disparity::ncc(s, dispMapNCC);
// 		newDisparityMap3=true;
// 	}
// }



int main(int argc, char* argv[])
{
	std::string tag = "MAIN\t";

	LOG(INFO) << tag << "Application started.";
	mvIMPACT::acquire::DeviceManager devMgr;

	Camera *left;
	Camera *right;

	initWindows();
	if(!Utility::initCameras(devMgr,left,right))
	{
		return 0;
	}


	baseline =131.97;
    fx = 615.00087800830659;
    fy = 614.52456559429913;
    cx = 375.75962528613366;
    cy = 232.47166090354159;

	std::vector<std::string> nodes;
  nodes.push_back("inputParameter");
  nodes.push_back("capturedDisparities");

  std::string config = "./configs/default.yml";

  cv::FileStorage fs;


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
    LOG(ERROR) << tag << "Unable to create directory for captured disparity maps." <<std::endl;
    return 0;
  }


	Stereosystem stereo(left,right);


  if(!stereo.loadIntrinsic(inputParameter+"/intrinsic.yml"))
    return 0;
  if(!stereo.loadExtrinisic(inputParameter +"/extrinsic.yml"))
    return 0;

	int key = 0;
 	int binning = 1;
	Stereopair s;
	double focusLeft, focusRight;
	
	//left->setBinning(binning);
	//right->setBinning(binning);
	stereo.resetRectification();

	disparitySGBM = cv::StereoSGBM(0,numDispSGBM,windSizeSGBM,8*windSizeSGBM*windSizeSGBM,32*windSizeSGBM*windSizeSGBM);
	disparityBM  = cv::StereoBM(CV_STEREO_BM_BASIC, numDispBM, windSizeBM);

	std::thread disp(disparityCalc,std::ref(s),std::ref(disparitySGBM));
	std::thread disp2(disparityCalcBM,std::ref(s),std::ref(disparityBM));
	//std::thread disp3(disparityCalcTM, std::ref(s));
	cv::Mat normalizedSGBM;
	cv::Mat normalizedBM;
	while(running)
	{
		stereo.getRectifiedImagepair(s);
		cv::imshow("Left", s.mLeft);
		cv::imshow("Right", s.mRight);


		if(newDisparityMap)
		{
			cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
			cv::imshow("SGBM",normalizedSGBM);
			newDisparityMap = false;
		}
		if(newDisparityMap2)
		{
			cv::normalize(dispMapBM,normalizedBM,0,255,cv::NORM_MINMAX, CV_8U);
			cv::imshow("BM",normalizedBM);
			newDisparityMap2 = false;
		}
		// if(newDisparityMap3)
		// {
		// 	cv::normalize(dispMapBM,normalizedBM,0,255,cv::NORM_MINMAX, CV_8U);
		// 	cv::imshow("Disparity2",normalizedBM);
		// 	newDisparityMap2 = false;
		// }
		cond_var.notify_one();

		key = cv::waitKey(10);

		if(char(key) == 'q')
		{
			running = false;
			cond_var.notify_one();
			break;
		}
		else if(char(key) == 'b')
		{
			if (binning == 0)
				binning = 1;
			else
				binning =0;

			left->setBinning(binning);
			right->setBinning(binning);
			left->setExposure(20000);
			right->setExposure(20000);
			stereo.resetRectification();
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

			{
				std::unique_lock<std::mutex> ul(disparityLockSGBM);
				cv::normalize(dispMapSGBM,normalizedSGBM,0,255,cv::NORM_MINMAX, CV_8U);
				cv::imwrite(std::string(outputDirectory+"/sgbm_raw_"+prefix+std::to_string(imageNumber)+".jpg"),dispMapSGBM);
				cv::imwrite(std::string(outputDirectory+"/sgbm_norm_"+prefix+std::to_string(imageNumber)+".jpg"),normalizedSGBM);
			}

			{
				std::unique_lock<std::mutex> ul(disparityLockBM);
				cv::normalize(dispMapBM,normalizedBM,0,255,cv::NORM_MINMAX, CV_8U);
				cv::imwrite(std::string(outputDirectory+"/bm_raw_"+prefix+std::to_string(imageNumber)+".jpg"),dispMapBM);
				cv::imwrite(std::string(outputDirectory+"/bm_norm_"+prefix+std::to_string(imageNumber)+".jpg"),normalizedBM);

			}
			++imageNumber;
		}
		else if(char(key) == 'f')
		{
			std::cout<<left->getFramerate()<<" "<<right->getFramerate()<<std::endl;
		}
	}

	disp.join();
	disp2.join();

	return 0;
}
