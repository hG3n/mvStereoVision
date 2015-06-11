#include "utility.h"

std::string mTag = "UTILITY\t";


// -----------------------------------------------------------------------------
// --- streopair ---------------------------------------------------------------
// -----------------------------------------------------------------------------
Stereopair::Stereopair():
  mLeft(),
  mRight(),
  mTag("STEREOPAIR\t")
{}

Stereopair::Stereopair(cv::Mat &l , cv::Mat &r):
  mLeft(l),
  mRight(r),
  mTag("STEREOPAIR\t")

{}
Stereopair::~Stereopair()
{
  mLeft.release();
  mRight.release();
  LOG(INFO) << mTag << "Stereopair destroyed" << std::endl;
}


// -----------------------------------------------------------------------------
// --- utility - directory -----------------------------------------------------
// -----------------------------------------------------------------------------
int Utility::getFiles (std::string const& dir, std::vector<std::string> &files)
{
  DIR *dp;
  struct dirent *dirp;

  //Unable to open dir
  if((dp  = opendir(dir.c_str())) == NULL)
  {
    std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    return errno;
  }

  //read files and push them to vector
  while ((dirp = readdir(dp)) != NULL)
  {
    std::string name = std::string(dirp->d_name);
    //discard . and .. from list
    if(name != "." && name != "..")
    {
      files.push_back(std::string(dirp->d_name));
    }
  }

  closedir(dp);
  std::sort(files.begin(), files.end());

  return 0;
}

bool Utility::directoryExist(std::string const& dirPath)
{
  struct stat st = {0};
  if(stat(dirPath.c_str(),&st) == 0)
  {
    return true;
  }
  return false;
}

bool Utility::createDirectory(std::string const& dirPath)
{
  system(std::string("mkdir -p " + dirPath).c_str());
  if(directoryExist(dirPath))
    return true;
  else
    return false;
}


// -----------------------------------------------------------------------------
// --- utility - init Cameras --------------------------------------------------
// -----------------------------------------------------------------------------
bool Utility::initCameras(mvIMPACT::acquire::DeviceManager &devMgr, Camera *&left, Camera *&right)
{
  const unsigned int devCnt = devMgr.deviceCount();

  if(devCnt != 2)
  {
    LOG(ERROR)<< mTag <<"Invalid numver of cameras detected! Number of detected cameras: " <<\
    devCnt << std::endl;
    return false;
  }


  if(devMgr[0]->serial.read() == "26803878")
  {
    if(devMgr[1]->serial.read() == "26803881")
    {
      LOG(INFO)<< mTag << "Successfully initialized both camers" <<std::endl;

      left = new Camera(devMgr[0]);
      right= new Camera(devMgr[1]);
      return true;
    }
    LOG(ERROR)<< mTag << "Error in camera initialization, got Serials:" <<\
    devMgr[0]->serial.read()<< " " <<devMgr[1]->serial.read()<<std::endl;
    return false;
  }

  if(devMgr[0]->serial.read() == "26803881")
  {
    if(devMgr[1]->serial.read() == "26803878")
    {
      LOG(INFO)<< mTag << "Successfully initilized both camers" <<std::endl;

      left = new Camera(devMgr[1]);
      right = new Camera(devMgr[0]);
      return true;
    }
    LOG(ERROR)<< mTag << "Error in camera initialization, got Serials:" <<\
    devMgr[0]->serial.read()<< " " <<devMgr[1]->serial.read()<<std::endl;
    return false;
  }
  return false;
}


// -----------------------------------------------------------------------------
// --- utility - config --------------------------------------------------------
// -----------------------------------------------------------------------------
bool Utility::checkConfig(std::string const& configfile, std::vector<std::string> const& nodes, cv::FileStorage &fs)
{
  bool success = fs.open(configfile, cv::FileStorage::READ);

  for(std::string currentNode : nodes)
  {
    if(fs[currentNode].empty())
    {
      LOG(ERROR) << mTag << "Node " << currentNode << " in " << configfile << " is empty." << std::endl;
      fs.release();
      return false;
    }
  }

  if(success)
  {
    LOG(INFO) << mTag <<"Successfully checked configuration." << std::endl;;
    return true;
   }
  else
  {
    LOG(ERROR) << mTag << "Unable to open configuration: " << configfile << std::endl;
    fs.release();
    return false;
  }
}


// -----------------------------------------------------------------------------
// --- utility - helper --------------------------------------------------------
// -----------------------------------------------------------------------------
double Utility::checkSharpness(cv::Mat const& src)
{
  cv::Mat M = (cv::Mat_<double>(3, 1) << -1, 2, -1);
  cv::Mat G = cv::getGaussianKernel(3, -1, CV_64F);
 
  cv::Mat Lx, Ly;
  cv::sepFilter2D(src, Lx, CV_64F, M, G);
  cv::sepFilter2D(src, Ly, CV_64F, G, M);

  cv::Mat FM = cv::abs(Lx) + cv::abs(Ly);
  return cv::mean(FM).val[0];
}

bool Utility::calcCoordinate(cv::Mat_<float> &toReturn,cv::Mat const& Q, cv::Mat const& disparityMap,int x,int y, int binning)
{
  double d = static_cast<float>(disparityMap.at<short>(y,x));

  d/=16.0;
  if(d > 0)
  {
    toReturn(0)=x;
    toReturn(1)=y;
    toReturn(2)=d;
    toReturn(3)=1;

    toReturn = Q*toReturn.t();
    toReturn/=toReturn(3);
    
    if(binning == 1)
      toReturn=toReturn/2;

    return true;
  }
  else
  {
    return false;
  }
}

float Utility::calcDistance(cv::Mat const& Q, float const& dispValue, int binning)
{
  float d = dispValue / 16;
  cv::Mat_<float> coordinateQ(1,4);

  coordinateQ(0)=1;
  coordinateQ(1)=1;
  coordinateQ(2)=d;
  coordinateQ(3)=1;

  if(binning == 0)
  {
    coordinateQ = (Q)*coordinateQ.t();
    coordinateQ/=coordinateQ(3);
  
    float distance = coordinateQ(2)/1000;
    coordinateQ.release();

    return distance;
  }
  else
  {
    coordinateQ = (Q/2)*coordinateQ.t();
    coordinateQ/=coordinateQ(3);
    
    float distance = coordinateQ(2)/1000;
    coordinateQ.release();

    // because binning is half of the image

    if(cvIsInf(distance))
    {
      return distance;
    }
    else
      return distance/2;
  }
}

void Utility::calcDistanceMap(cv::Mat &distanceMap, cv::Mat const& dMap, cv::Mat const& Q, int binning)
{
  //TODO:
  // although it is inefficient and slow as ...
  // TODO: fix crash when copying the distances into the mat
  distanceMap = cv::Mat(dMap.rows, dMap.cols, CV_32F);
  for(int r = 0; r < dMap.rows; ++r)
  {
    for(int c = 0; c < dMap.cols; ++c)
    {
      cv::Mat_<float> coord(1,4);
      coord = calcCoordinate(coord, Q, dMap, c,r,binning);
      distanceMap.at<float>(c,r) = 0;
      coord.release();
    }
  } 
}

float Utility::calcMeanDisparity(cv::Mat const& matrix)
{
  int total = 0;
  int numElements = 0;
  for(int r = 0; r < matrix.rows; ++r)
  {
    for(int c = 0; c < matrix.cols; ++c)
    {
      if(static_cast<float>(matrix.at<short>(r,c)) > 0)
      {
        total += static_cast<float>(matrix.at<short>(r,c));
        ++numElements;
      }
    }
  } 
  // if the matrix appears to be empty because of any reason
  // return disparity of 1.0 do indicate infinity
  if(total == 0 || numElements == 0){
    return 0.0;
  }
  else
  {
    float mean = total / abs(numElements);
    return mean;
  }
}

std::pair<float,float> Utility::calcMinMaxDisparity(cv::Mat const& matrix)
{
  std::vector<float> elements;
  for(int r = 0; r < matrix.rows; ++r)
  {
    for(int c = 0; c < matrix.cols; ++c)
    {
      if(static_cast<float>(matrix.at<short>(r,c)) > 0)
      {
        float current = static_cast<float>(matrix.at<short>(r,c));
        elements.push_back(current);
      }
    }
  } 
  auto min = std::min_element(std::begin(elements), std::end(elements));
  auto max = std::max_element(std::begin(elements), std::end(elements));
  return std::make_pair(*min,*max);
}
