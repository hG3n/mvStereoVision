#ifndef __Camera__H__
#define __Camera__H__

#include <vector>
#include <iostream>
#include <string>

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#include "easylogging++.h"

#include "opencv2/opencv.hpp"

//Pixelformat
#define MONO8 0
#define MONO16 1

//Binning
#define BINNING_OFF 0
#define BINNING_HV 1


class Camera
{
	public:
		//Constructor, Destructor
						Camera();
						Camera(mvIMPACT::acquire::Device*);
						~Camera();

		//Getter
		float 				getFramerate() 		const;
		unsigned int 	getImageWidth() 	const;
		unsigned int 	getImageHeight() 	const;
		int 					getExposure() 		const;
		float 				getGain() 				const;
		int 					getBinningMode()	const;
		cv::Mat 			getIntrinsic() 		const;
		cv::Mat 			getDistCoeffs() 	const;

		//Setter
		void 			setExposure(unsigned int);
		void 			setGain(float);
		void 			setPixelFormat(int);
		void 			setBinning(unsigned int);
		void			setIntrinsic(cv::Mat);

		//Functions
		bool 				getImage(std::vector<char>&);
		double 			calibrate(std::vector<cv::Mat> const&, double, cv::Size);

	private:
		//Camera settings
		mvIMPACT::acquire::Device* 								mDevice;
		mvIMPACT::acquire::FunctionInterface			mFunctionInterface;
	 	mvIMPACT::acquire::Statistics 						mStatistics;
  	mvIMPACT::acquire::SystemSettings 				mSystemSettings;
  	mvIMPACT::acquire::CameraSettingsBase 		mCameraSettingsBase;
  	mvIMPACT::acquire::CameraSettingsBlueFOX 	mCameraSettingsBlueFOX;
  	mvIMPACT::acquire::ImageDestination 			mImageDestinaton;
		mvIMPACT::acquire::Request*								mRequest;
		int 																			mTimeout;

		//Log tag
		std::string									mTag;

		//image width and height
		unsigned int 								mWidth;
		unsigned int 								mHeight;
		int 												mBinningMode;

		// intrinsics of a camera
		cv::Mat mIntrinsic;
		cv::Mat mDistCoeffs;
};

#endif //__Camera__H__