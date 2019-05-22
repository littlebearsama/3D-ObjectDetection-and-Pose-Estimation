/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_LOOP_CLOSING_RT_HH
#define KP_LOOP_CLOSING_RT_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Dense>
#include "Scene.hh"
#include "View.hh"
#include "v4r/KeypointTools/RigidTransformationRANSAC.hh"
#include "v4r/KeypointTools/SmartPtr.hpp"

namespace kp
{

class LoopClosingRT 
{
public:
  class Parameter
  {
  public:
    double max_angle;
    double max_distance;
    double min_total_score;         // minimum score (matches weighted with inv. inl. dist)
    int min_tiles_used;
    RigidTransformationRANSAC::Parameter rt_param;
    Parameter(double _max_angle=30, double _max_distance=2., double _min_total_score=10., 
      int _min_tiles_used=3,
      const RigidTransformationRANSAC::Parameter &_rt_param=RigidTransformationRANSAC::Parameter(0.005) )
    : max_angle(_max_angle), max_distance(_max_distance), min_total_score(_min_total_score), 
      min_tiles_used(_min_tiles_used),
      rt_param(_rt_param) {}
  };

private:
  Parameter param;
  
  Scene::Ptr scene;

  double sqr_max_distance, rad_max_angle;

  cv::Mat_<cv::Vec3b> image;
  cv::Mat_<unsigned char> im_gray;

  std::vector<Eigen::Vector3f> src_pts;
  std::vector<Eigen::Vector3f> tgt_pts;

  RigidTransformationRANSAC::Ptr rt;
  SmartPtr<cv::DescriptorMatcher> matcher;

  Eigen::Vector3f computeCenter(const View &frame);
  double getDeltaViewingAngle(View &frame1, const View &frame2);
  bool isTrackOK(int num_tiles, const double &total_score, const std::vector<double> &score_per_tile);
  void detectHypotheses(std::vector<View::Ptr> &views, View &view, std::vector<int> &indices);
  void matchHypotheses(std::vector<View::Ptr> &views, View &view, std::vector<int> &indices, 
        std::vector<std::vector<cv::DMatch> > &matches);
  int verifyMatchesRT(std::vector<View::Ptr> &views, View &view, std::vector<int> &indices, 
        std::vector<std::vector<cv::DMatch> > &matches, std::vector<Eigen::Matrix4f> &poses);
  bool insertKeypointLinks(View &kf, View &view, std::vector<cv::DMatch> &matches);
  double getDeltaSqrPoseDistance(View &frame1, const View &frame2);

  


public:
  cv::Mat dbg;

  LoopClosingRT(const Parameter &p=Parameter());
  ~LoopClosingRT();

  void detectLoops(View &frame);

  Scene::Ptr &getSharedData() {return scene;}
  void setSharedData(Scene::Ptr &_scene) {scene = _scene;}

  typedef SmartPtr< ::kp::LoopClosingRT> Ptr;
  typedef SmartPtr< ::kp::LoopClosingRT const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

