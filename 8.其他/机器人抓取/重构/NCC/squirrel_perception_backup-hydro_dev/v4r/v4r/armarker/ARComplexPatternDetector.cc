/**
 * $Id$
 */


#include "ARComplexPatternDetector.hh"

#ifndef ARC_DEBUG
#define ARC_DEBUG
#endif


namespace ar 
{

using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
ARComplexPatternDetector::ARComplexPatternDetector(const string &config_file, int _paramMinPoints)
 : paramMinPoints(_paramMinPoints)
{ 
  setup(config_file);
}

ARComplexPatternDetector::~ARComplexPatternDetector()
{
}


/**
 * setup
 * file format:
 *   number_of_marker
 *   marker_file_name
 *   x y z   # 3d coordinates of the upper right point [1 1]
 *   x y z   # 3d coordinates of the lower right point [1 0]
 *   x y z   # 3d coordinates of the lower left point [0 0]
 *   x y z   # 3d coordinates of the upper left point [0 1]
 *   ...
 */
void ARComplexPatternDetector::setup(const string &config_file)
{
  int num, id, z=0;;
  string file;
  cv::Point3f pt;

  ifstream in(config_file.c_str());

  if (in.is_open())
  {
    in>>num;

    while(!in.eof() && z<num)
    {
      in>>file;

      id = ardetector.loadMarker(file);
  
      if (id<0)
        throw runtime_error("[ARComplexPatternDetector::Setup] Marker file not found!");

      for (unsigned i=0; i<4; i++)
      {
        in>> pt.x >> pt.y >> pt.z;
        markerPoints[id].push_back(pt);
      }

      z++;
    }
    
  }
  else throw runtime_error("[ARComplexPatternDetector::Setup] Config file not found!");
}






/******************************* PUBLIC ***************************************/

/**
 * Detect
 */
void ARComplexPatternDetector::detectMarker(const cv::Mat &image, std::vector<cv::Point3f> &objPoints, std::vector<cv::Point2f> &imgPoints)
{
  objPoints.clear();
  imgPoints.clear();

  std::vector<V4R::Marker> marker = ardetector.detectMarker(image);

  for (unsigned i=0; i<marker.size(); i++)
  {
    V4R::Marker &m = marker[i];

    #ifdef ARC_DEBUG
    //cout<<"id="<<m.id<<endl;
    #endif

    if (m.id != -1 && m.id<(int)markerPoints.size())
    {
      std::vector<cv::Point3f> &pts = markerPoints[m.id];
      
      for (unsigned j=0; j<pts.size(); j++)
      {
        imgPoints.push_back(cv::Point2f(m.vertex[j][0], m.vertex[j][1]));    // 0 0
        objPoints.push_back(pts[j]);
      }

      #ifdef ARC_DEBUG
      if (!dbg.empty())
      {
        cv::line(dbg, cv::Point2f(m.vertex[2][0], m.vertex[2][1]), cv::Point2f(m.vertex[1][0], m.vertex[1][1]), CV_RGB(255,0,0), 2);
        cv::line(dbg, cv::Point2f(m.vertex[2][0], m.vertex[2][1]), cv::Point2f(m.vertex[3][0], m.vertex[3][1]), CV_RGB(0,255,0), 2);
      }
      #endif

    }
  }
}


/**
 * GetPose
 */
bool ARComplexPatternDetector::getPose(const cv::Mat &image, const cv::Mat& intrinsic, const cv::Mat& _distCoeffs, cv::Mat &R, cv::Mat &T)
{
  std::vector<cv::Point3f> objPoints;
  std::vector<cv::Point2f> imgPoints;
  std::vector<int> inlIndices;

  cv::Mat rod;
  cv::Mat distCoeffs = _distCoeffs;

  if (distCoeffs.empty())
    distCoeffs = cv::Mat::zeros(1,4,CV_64F);

  detectMarker(image, objPoints, imgPoints);

  if (objPoints.size()>=(unsigned)paramMinPoints)
  {
    //cv::solvePnP(cv::Mat(objPoints), cv::Mat(imgPoints), intrinsic, distCoeffs, rod, T, false);
    //ransac pose
    cv::solvePnPRansac(cv::Mat(objPoints), cv::Mat(imgPoints), intrinsic, distCoeffs, 
          rod, T, false, 50, 3., 20, inlIndices);
    
    R= cv::Mat(3,3,CV_64F);
    cv::Rodrigues(rod,R);

    if (inlIndices.size()<(unsigned)paramMinPoints)
      return false;
    return true;
  }

  return false;
}



}












