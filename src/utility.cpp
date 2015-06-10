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
bool Utility::calcCoordinate(cv::Mat_<float> &toReturn,cv::Mat const& Q, cv::Mat const& disparityMap,int x,int y)
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
      return true;
    }
    else
    {
      return false;
    }
}

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

