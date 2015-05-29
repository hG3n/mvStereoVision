#include "obstacleDetection.h"
#include "utility.h"

obstacleDetection::obstacleDetection():
  mDispMap(),
  mTag("OBSTACLE DETECTION\t"),
  mBinning(0),
  mSamplepoints(),
  mSubimages(),
  mDistanceMapMean(),
  mDistanceMapMin(),
  mDistanceMapStdDev(),
  mMeanMap()
{}

obstacleDetection::obstacleDetection(cv::Mat const& disparityMap, int binning):
  mDispMap(disparityMap),
  mTag("OBSTACLE DETECTION\t"),
  mBinning(binning),
  mSamplepoints(),
  mSubimages(),
  mDistanceMapMean(),
  mDistanceMapMin(),
  mDistanceMapStdDev(),
  mMeanMap()
{
  //built the Subimage 'tree'
  Subimage sub = Subimage(mDispMap, 0);
  sub.subdivide();
  mSubimages = sub.getSubdividedImages();
  for (unsigned int i = 0; i < mSubimages.size(); ++i)
  {
    mSubimages[i].subdivide();
  }

  // LOG(INFO)<< mTag <<"Obstacle Detection created\n";
}

obstacleDetection::~obstacleDetection()
{
  mDispMap.release();
  // LOG(INFO)<< mTag <<"Obstacle Detection destroyed\n";
}


std::vector<std::vector<float>> obstacleDetection::getMeanMap() const
{
  return mMeanMap;
}

std::vector<std::vector<float>> obstacleDetection::getDistanceMapMean() const
{
  return mDistanceMapMean;
}

std::vector<std::vector<float>> obstacleDetection::getDistanceMapMin() const
{
  return mDistanceMapMin;
}

std::vector<std::vector<float>> obstacleDetection::getDistanceMapStdDev() const
{
  return mDistanceMapStdDev;
}

std::vector<Subimage> obstacleDetection::getSubimages() const
{
  return mSubimages;
}


void obstacleDetection::update(cv::Mat const& disparityMap, int)
{
  mDispMap = disparityMap;
  
}

void obstacleDetection::buildMeanMap(cv::Mat const& Q)
{
  unsigned int numSubimages = mSubimages.size();
  if (numSubimages == 0)
  {
    LOG(INFO)<< mTag <<"Unable to build Mean-Map. No Subimages provided\n";
  }
  
  std::vector<float> meanStorage;
  for (unsigned int i = 0; i < numSubimages; ++i)
  {
    for (unsigned int j = 0; j < numSubimages; ++j)
    {
      float mean = mSubimages[i].getSubdividedImages()[j].calcMean();
      meanStorage.push_back(mean);
    }
    mMeanMap.push_back(meanStorage);
    meanStorage.clear();
  }
}

void obstacleDetection::buildMeanDistanceMap(cv::Mat const& Q)
{
  unsigned int numSubimages = mSubimages.size();
  if (numSubimages == 0)
  {
    LOG(INFO)<< mTag <<"Unable to build Mean-Distance-Map. No Subimages provided\n";
  }

  std::vector<float> distanceStorage;
  for (unsigned int i = 0; i < numSubimages; ++i)
  {
    for (unsigned int j = 0; j < numSubimages; ++j)
    {
      float mean = mSubimages[i].getSubdividedImages()[j].calcMean();
      float distance = Utility::calcDistance(Q, mean,mBinning);
      distanceStorage.push_back(distance);
    }
    mDistanceMapMean.push_back(distanceStorage);
    distanceStorage.clear();
  }
}


void obstacleDetection::buildMinDistanceMap(cv::Mat const& Q)
{
  unsigned int numSubimages = mSubimages.size();
  if (numSubimages == 0)
  {
    LOG(INFO)<< mTag <<"Unable to build Min-Distance-Map. No Subimages provided\n";
  }

  std::vector<float> distanceStorage;
  for (unsigned int i = 0; i < numSubimages; ++i)
  {
    for (unsigned int j = 0; j < numSubimages; ++j)
    {
      //have to use the max value because of distance calculation
      float min = mSubimages[i].getSubdividedImages()[j].calcMinMax().second;
      float distance = Utility::calcDistance(Q, min, mBinning);
      distanceStorage.push_back(distance);
    }
    mDistanceMapMin.push_back(distanceStorage);
    distanceStorage.clear();
  }
}

void obstacleDetection::buildStdDevDistanceMap(cv::Mat const& Q)
{
  unsigned int numSubimages = mSubimages.size();
  if (numSubimages == 0)
  {
    LOG(INFO)<< mTag <<"Unable to build Min-Distance-Map. No Subimages provided\n";
  }

  std::vector<float> distanceStorage;
  for (unsigned int i = 0; i < numSubimages; ++i)
  {
    for (unsigned int j = 0; j < numSubimages; ++j)
    {
      //have to use the max value because of distance calculation
      float stddev = mSubimages[i].getSubdividedImages()[j].calcMeanStdDev().first[0];
      //float distance = Utility::calcDistance(Q, min, mBinning);
      distanceStorage.push_back(stddev);
    }
    mDistanceMapStdDev.push_back(distanceStorage);
    distanceStorage.clear();
  }
}

void obstacleDetection::detectObstacles(int const& mode, std::pair<float,float> const& threshold)
{

  if (mode == MEAN_DISTANCE)
  {
    for (unsigned int i = 0; i < mDistanceMapMean.size(); ++i)
    {
      for (unsigned int j = 0; j < mDistanceMapMean[i].size(); ++j)
      {
        float minBorder = threshold.first;
        float maxBorder = threshold.second;
        float value = mDistanceMapMean[i][j];
        if (value > minBorder && value < maxBorder)
        {
          std::cout << "Obstacle Detected in: " << i << " Subimage:" << j << std::endl;
        }
      }
    }
  }
  else if (mode == MIN_DISTANCE)
  {
   for (unsigned int i = 0; i < mDistanceMapMean.size(); ++i)
    {
      for (unsigned int j = 0; j < mDistanceMapMean[i].size(); ++j)
      {
        float minBorder = threshold.first;
        float maxBorder = threshold.second;
        float value = mDistanceMapMin[i][j];
        if (value > minBorder && value < maxBorder)
        {
          std::cout << "Obstacle Detected in: " << i << " Subimage:" << j << std::endl;
        }
      }
    }
  }
  else if (mode == STDDEV)
  {
   for (unsigned int i = 0; i < mDistanceMapMean.size(); ++i)
    {
      for (unsigned int j = 0; j < mDistanceMapMean[i].size(); ++j)
      {
        float minBorder = threshold.first;
        float maxBorder = threshold.second;
        float value = mDistanceMapMin[i][j];
        if (value > minBorder && value < maxBorder)
        {
          std::cout << "Obstacle Detected in: " << i << " Subimage:" << j << std::endl;
        }
      }
    }
  }
}
