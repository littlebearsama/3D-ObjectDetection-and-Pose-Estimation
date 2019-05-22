/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_MARKER_DETECTOR_HH
#define KP_MARKER_DETECTOR_HH

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <map>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/armarker/armarker.h"
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "Scene.hh"
#include "View.hh"


namespace kp
{


/**
 * ARMarkerDetector
 */
class ARMarkerDetector
{
public:
  class Parameter
  {
  public:
    float reprojection_error;
    Parameter(float _reprojection_error=3.) : reprojection_error(_reprojection_error) {}
  };

private:
  Parameter param;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;
 
  std::map<int, std::vector<cv::Point3f> > pts_marker;
  std::map<int, std::vector<int> > pts_global;

  Scene::Ptr scene;

  V4R::MarkerDetection ardetector;

  void setup(const std::string &config_file);
  void addToView(int id, const std::vector<cv::Point2f> &pts_image);
  void filter(Scene &scene, View &view);


  inline void cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose); 



public:
  cv::Mat dbg;

  ARMarkerDetector(const std::string &config_file, const Parameter &p=Parameter());
  ~ARMarkerDetector();

  void detect(const cv::Mat_<cv::Vec3b> &image);
  void reconstruct();

  void setSharedData(Scene::Ptr &_scene) {scene = _scene;}
  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef SmartPtr< ::kp::ARMarkerDetector> Ptr;
  typedef SmartPtr< ::kp::ARMarkerDetector const> ConstPtr;
};


/***************************** inline methods *******************************/
/**
 * cvToEigen
 */
inline void ARMarkerDetector::cvToEigen(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose)
{
  pose.setIdentity();

  pose(0,0) = R(0,0); pose(0,1) = R(0,1); pose(0,2) = R(0,2);
  pose(1,0) = R(1,0); pose(1,1) = R(1,1); pose(1,2) = R(1,2);
  pose(2,0) = R(2,0); pose(2,1) = R(2,1); pose(2,2) = R(2,2);

  pose(0,3) = t(0,0);
  pose(1,3) = t(1,0);
  pose(2,3) = t(2,0);
}





} //--END--

#endif

