/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_TRIANGULATION_HH
#define KP_TRIANGULATION_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "View.hh"
#include "Scene.hh"

namespace kp
{

class Triangulation 
{
public:
  class Parameter
  {
  public:
    float reprojection_error;
    float reprojection_error2; 
    float min_angle;
    Parameter(float _reprojection_error=1., float _reprojection_error2=3., float _min_angle=5.) 
    : reprojection_error(_reprojection_error), reprojection_error2(_reprojection_error2), min_angle(_min_angle) {}
  };

private:
  Parameter param;

  float min_angle_rad;
  
  Scene::Ptr scene;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;

  std::vector<cv::Point2f> pts_kf, pts_view;

  void getPoints(View &keyframe, View &view, 
        std::vector<cv::Point2f> &pts_kf, std::vector<cv::Point2f> &pts_view, 
        std::vector< std::pair<int, int> > &kf_view);
  void triangulatePoints(std::vector<cv::Point2f> &pts_kf, std::vector<cv::Point2f> &pts_view, 
        std::vector< std::pair<int, int> > &kf_view_idx, View &keyframe, View &view);

  inline void eigenToCv(const Eigen::Matrix<float, 3, 4> &eig_mat, cv::Mat_<float> &cv_mat); 



public:
  cv::Mat dbg;

  Triangulation(const Parameter &p=Parameter());
  ~Triangulation();

  void filter(Scene &scene, View &view);
  int triangulate();

  Scene::Ptr &getSharedData() {return scene;}
  void setSharedData(Scene::Ptr &_scene) {scene = _scene;}

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef SmartPtr< ::kp::Triangulation> Ptr;
  typedef SmartPtr< ::kp::Triangulation const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

inline void Triangulation::eigenToCv(const Eigen::Matrix<float, 3, 4> &eig_mat, cv::Mat_<float> &cv_mat)
{
  cv_mat = cv::Mat_<float>(3,4);
  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<4; j++)
      cv_mat(i,j) = eig_mat(i,j);  
}



} //--END--

#endif

