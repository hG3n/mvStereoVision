#include "disparity.h"
#include "utility.h"
#include "math.h"

// Semi Global Block matching 
void Disparity::sgbm(Stereopair const& inputImages, cv::Mat &output, cv::StereoSGBM dispCompute)
{
	dispCompute(inputImages.mLeft,inputImages.mRight,output);
	//output/=16;
}

// simple blockmatching 
void Disparity::bm(Stereopair const& inputImages, cv::Mat &output, cv::StereoBM dispCompute)
{
  dispCompute(inputImages.mLeft,inputImages.mRight,output);
  //output/=16;
}

// self written template matching algorithm
void Disparity::tm(Stereopair const& inputImages, cv::Mat &output, unsigned int kernelSize)
{
  output = cv::Mat(inputImages.mLeft.rows, inputImages.mLeft.cols,CV_8U,cv::Scalar::all(0));

  cv::Mat currentTemplate;
  cv::Mat currentSearchRange;
  cv::Mat result;

  double minVal;
  double maxVal;
  cv::Point2i minLoc;
  cv::Point2i maxLoc;
  // cv::Point2i matchLoc;

  for(int i = 0; i < inputImages.mLeft.rows-kernelSize; ++i)
  {
    for(int j = 0; j < inputImages.mLeft.cols-kernelSize ; ++j)
    {
      cv::Point2i position(j,i);
      cv::Rect kernel(position, cv::Size(kernelSize,kernelSize));

      currentTemplate = inputImages.mLeft(kernel);
      int destWidth = inputImages.mRight.cols - position.x-1;
      currentSearchRange = inputImages.mRight(cv::Rect(position,cv::Size(destWidth,kernelSize)));

      cv::matchTemplate(currentSearchRange,currentTemplate,result,CV_TM_CCORR_NORMED);

      cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

      output.at<char>(i,j) = maxLoc.x;
      //std::cout<< int(output.at<char>(i,j)) <<std::endl;

    }
  }

}
