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

template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line)
{
  //Line is given as a*x + b*y + c = 0
  return std::fabs(line(0)*point.x + line(1)*point.y + line(2))
      / std::sqrt(line(0)*line(0)+line(1)*line(1));
}

void drawEpipolarLines(const std::string& title, const cv::Mat F,
                cv::Mat& img1, cv::Mat& img2,
                const std::vector<cv::Point2f> points1,
                const std::vector<cv::Point2f> points2,
                const float inlierDistance = -1)
{
  CV_Assert( img1.size() == img2.size() && img1.type() == img2.type());
  cv::Mat outImg(img1.rows, img1.cols*2, CV_8UC3);
  cv::Rect rect1(0,0, img1.cols, img1.rows);
  cv::Rect rect2(img2.cols, 0, img2.cols, img2.rows);
  /*
   * Allow color drawing
   */
  if (img1.type() == CV_8U)
  {
    cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
    cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
  }
  else
  {
    img1.copyTo(outImg(rect1));
    img2.copyTo(outImg(rect2));
  }
  std::vector<cv::Vec<float,3>> epilines1, epilines2;
  cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
  cv::computeCorrespondEpilines(points2, 2, F, epilines2);


  cv::RNG rng(0);
  for(size_t i=0; i<points1.size(); i++)
  {
    if(inlierDistance > 0)
    {
      if(distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
        distancePointLine(points2[i], epilines1[i]) > inlierDistance)
      {
        //The point match is no inlier
        continue;
      }
    }
    /*
     * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
     */
    cv::Scalar color(0,255,0);
    cv::Mat rec2 = outImg(rect2); 
    cv::Mat rec1 = outImg(rect1);

    cv::line(rec2, cv::Point(0,-epilines1[i][2]/epilines1[i][1]), cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]), color,1);
    cv::circle(rec1, points1[i], 3, color, -1, CV_AA);
    
    cv::line(img2, cv::Point(0,-epilines1[i][2]/epilines1[i][1]), cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]), color,1);
    cv::circle(img1, points1[i], 3, color, -1, CV_AA);
 
    cv::line(img1,cv::Point(0,-epilines2[i][2]/epilines2[i][1]),cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),color,1);
    cv::circle(img2, points2[i], 3, color, -1, CV_AA);

    cv::line(rec1,cv::Point(0,-epilines2[i][2]/epilines2[i][1]),cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),color,1);
    cv::circle(rec2, points2[i], 3, color, -1, CV_AA);
  }
  cv::imshow(title, outImg);
}
 


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


	if(!stereo.loadIntrinsic("parameters/intrinsic.yml"))
		return 0;
	if(!stereo.loadExtrinisic("parameters/extrinsic.yml"))
		return 0;

	Stereopair s;

	
	std::string dirPath = "capturedImages";
	std::string pathLeft = dirPath+"/left";
	std::string pathRight = dirPath+"/right";

	std::system(std::string("mkdir -p " + pathLeft).c_str());
	std::system(std::string("mkdir -p " + pathRight).c_str());
	
	if(Utility::directoryExist(pathLeft) && Utility::directoryExist(pathRight))
	{
			LOG(INFO) << tag << "Successfully created directories for captured images." << std::endl; 
	}
	else
	{
		LOG(ERROR) << tag << "Unable to create directories for captured images." <<std::endl;
		return 0;
	}

	char key = 0;
 	int binning = 0;
	int imageCounter = 0;
	cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Right", cv::WINDOW_AUTOSIZE);
	bool running = true;
	std::string filename = "";

	std::vector<cv::Point2f> p1;
	std::vector<cv::Point2f> p2;

	p1.push_back(cv::Point2f(100,100));
	p2.push_back(cv::Point2f(100,100));
	p1.push_back(cv::Point2f(200,200));
	p2.push_back(cv::Point2f(200,200));

	cv::Mat fundamental;
	stereo.getFundamentalMatrix(fundamental);
	while(running)
	{
	
		if(!stereo.getRectifiedImagepair(s))
		{
			break;
		}
		cv::imshow("Left", s.mLeft);
		cv::imshow("Right", s.mRight);
	
		key = cv::waitKey(5);
	
		drawEpipolarLines("Test",fundamental,s.mLeft,s.mRight,p1,p2 );

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
					if(imageCounter < 10)
					{
						filename+= "00" + std::to_string(imageCounter);
					}
					if(imageCounter >= 10 && imageCounter < 100)
					{
						filename+="0" + std::to_string(imageCounter);
					}

					cv::imwrite(pathLeft+"/left_"+filename+".jpg",s.mLeft);
					cv::imwrite(pathRight+"/right_"+filename+".jpg",s.mRight);
					LOG(INFO) << tag << "Wrote left image to " << std::string(pathLeft+"/left_"+filename+".jpg") <<std::endl;
					LOG(INFO) << tag << "Wrote right image to " << std::string(pathRight+"/right_"+filename+".jpg") <<std::endl;
					++imageCounter;
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