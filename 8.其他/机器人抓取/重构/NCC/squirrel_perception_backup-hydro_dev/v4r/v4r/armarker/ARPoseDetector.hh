/**
 * $Id$
 */

#ifndef AR_POSE_DETECTOR_HH
#define AR_POSE_DETECTOR_HH

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <map>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "armarker.h"


namespace ar 
{

/**
 * Marker
 */
class Marker
{ 
public:
  int id;
  Eigen::Matrix4f pose;
  std::vector<Eigen::Vector3f> model_points;
  std::vector<Eigen::Vector2f> image_points;
  Marker(int _id=-1) : id(_id){}
};

/**
 * ARPoseDetector
 */
class ARPoseDetector
{
private:
  cv::Mat intrinsic;
  cv::Mat dist_coeffs;

  V4R::MarkerDetection ardetector;
  std::vector<cv::Point3f> model_points;
  
  inline void cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose);

public:
  cv::Mat dbg;

  ARPoseDetector(const double &_sqr_size);
  ~ARPoseDetector();

  int addMarker(const std::string &_pattern_name);

  void detectMarker(const cv::Mat &image, std::map<int, std::vector<cv::Point2f> > &im_pts);
  void detectPose(const cv::Mat &image, std::vector<Marker> &marker); 

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs=cv::Mat() );

  typedef boost::shared_ptr< ::ar::ARPoseDetector> Ptr;
  typedef boost::shared_ptr< ::ar::ARPoseDetector const> ConstPtr;
};


/**
 * cvToEigen
 */
inline void ARPoseDetector::cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose)
{
  pose.setIdentity();
  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<3; j++)
      pose(i,j) = R(i,j);
  for (unsigned i=0; i<3; i++)
    pose(i,3) = t(i);
}


} //--END--

#endif

