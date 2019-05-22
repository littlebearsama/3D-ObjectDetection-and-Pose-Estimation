/**
 * $Id$
 */


#include "ARPoseDetector.hh"


namespace ar 
{

using namespace std;


/************************************************************************************
 * Constructor/Destructor
 * @param _pattern_name
 * @param _sqr_size defines the squared pattern:
 *   x y z   # 3d coordinates of the upper right point [1 1]
 *   x y z   # 3d coordinates of the lower right point [1 0]
 *   x y z   # 3d coordinates of the lower left point [0 0]
 *   x y z   # 3d coordinates of the upper left point [0 1]
 */
ARPoseDetector::ARPoseDetector(const double &_sqr_size)
{ 
  model_points.push_back(cv::Point3f(_sqr_size, _sqr_size, 0.));
  model_points.push_back(cv::Point3f(_sqr_size, 0., 0.));
  model_points.push_back(cv::Point3f(0., _sqr_size, 0.));
  model_points.push_back(cv::Point3f(0., _sqr_size, 0.));
}

ARPoseDetector::~ARPoseDetector()
{
}




/******************************* PUBLIC ***************************************/

/**
 * detecttMarker
 */
void ARPoseDetector::detectMarker(const cv::Mat &image,  std::map<int, std::vector<cv::Point2f> > &im_pts)
{
  im_pts.clear();

  std::vector<V4R::Marker> marker = ardetector.detectMarker(image);

  for (unsigned i=0; i<marker.size(); i++)
  {
    V4R::Marker &m = marker[i];

    if (m.id != -1)
    {
      std::vector<cv::Point2f> &pts = im_pts[m.id];
      pts.clear();

      for (unsigned j=0; j<4; j++)
      {
        pts.push_back( cv::Point2f(m.vertex[j][0], m.vertex[j][1]) );    // 0 0
      }

      if (!dbg.empty())
      {
        cv::line(dbg, cv::Point2f(m.vertex[2][0], m.vertex[2][1]), cv::Point2f(m.vertex[1][0], m.vertex[1][1]), CV_RGB(255,0,0), 2);
        cv::line(dbg, cv::Point2f(m.vertex[2][0], m.vertex[2][1]), cv::Point2f(m.vertex[3][0], m.vertex[3][1]), CV_RGB(0,255,0), 2);
      }
    }
  }
}

/**
 * addMarker
 */
int ARPoseDetector::addMarker(const std::string &_pattern_name)
{
  return ardetector.loadMarker(_pattern_name);
}

/** 
 * detect
 */
void ARPoseDetector::detectPose(const cv::Mat &image, std::vector<Marker> &marker)
{
  cv::Mat_<double> R(3,3), T(3,1), rod;
  std::map<int, std::vector<cv::Point2f> > im_pts;
  std::map<int, std::vector<cv::Point2f> >::iterator it;

  detectMarker(image, im_pts);

  for (it = im_pts.begin(); it!=im_pts.end(); it++)
  {
    std::vector<cv::Point2f> &pts = it->second;

    cv::solvePnP(cv::Mat(model_points), cv::Mat(pts), intrinsic, dist_coeffs, rod, T, false);
    cv::Rodrigues(rod,R);

    marker.push_back(Marker(it->first));
    Marker &m = marker.back();

    cvToEigen(R,T, m.pose);
    for (unsigned i=0; i<4; i++)
    {
      m.image_points.push_back(Eigen::Vector2f(pts[i].x,pts[i].y));
      m.model_points.push_back(Eigen::Vector3f(model_points[i].x,model_points[i].y,model_points[i].z));
    }
  }
}

/**
 * setCameraParameter
 */
void ARPoseDetector::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  dist_coeffs = cv::Mat_<double>();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(intrinsic, CV_64F);
  else intrinsic = _intrinsic;

  if (!_dist_coeffs.empty())
  {
    if (_dist_coeffs.type() != CV_64F)
      _dist_coeffs.convertTo(dist_coeffs, CV_64F);
    else dist_coeffs = _dist_coeffs;
  }
}


}












