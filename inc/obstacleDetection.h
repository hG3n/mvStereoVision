#ifndef __OBSTACLE_DETECION__H
#define __OBSTACLE_DETECION__H

#include <vector>
#include <iostream>

//OPENCV Stuff
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "subimage.h"

#define MEAN_DISTANCE 0
#define MIN_DISTANCE 1
#define STDDEV 2
#define SAMPLE 3

class obstacleDetection
{
  public:
    obstacleDetection();
    obstacleDetection(cv::Mat const&, int);
    ~obstacleDetection();

    std::vector<std::vector<float>> getMeanMap() const;
    std::vector<std::vector<float>> getDistanceMapMean() const;
    std::vector<std::vector<float>> getDistanceMapMin() const;
    std::vector<std::vector<float>> getDistanceMapStdDev() const;
    std::vector<Subimage> getSubimages() const;

    void update(cv::Mat const&, int);

    void buildMeanMap(cv::Mat const&);

    void buildMeanDistanceMap(cv::Mat const&);
    void buildMinDistanceMap(cv::Mat const&);
    void buildStdDevDistanceMap(cv::Mat const&);

    void detectObstacles(int const&, std::pair<float,float> const&);

  private:
    cv::Mat                         mDispMap;
    std::string                     mTag;
    int                             mBinning;
    std::vector<cv::Point>          mSamplepoints;
    std::vector<Subimage>           mSubimages;
    std::vector<std::vector<float>> mDistanceMapMean;
    std::vector<std::vector<float>> mDistanceMapMin;
    std::vector<std::vector<float>> mDistanceMapStdDev;
    std::vector<std::vector<float>> mMeanMap;
};

#endif //__OBSTACLE_DETECION__H