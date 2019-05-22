/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CAMERA_TRACKER_HH
#define KP_CAMERA_TRACKER_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include "v4r/KeypointTools/RigidTransformationRANSAC.hh"
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "v4r/KeypointTools/PointTypes.hpp"
#include "KeypointTracker.hh"
#include "LoopClosingRT.hh"
#ifndef KP_NO_CERES_AVAILABLE
#include "BundleAdjusterRT.hh"
#endif
#include "Scene.hh"
#include "View.hh"

namespace kp
{

class CameraTrackerRGBD 
{
public:
  class Parameter
  {
  public:
    double min_total_score;         // minimum score (matches weighted with inv. inl. dist)
    int min_tiles_used;          // minimum number of tiles with matches
    double angle_init_keyframe; 
    bool detect_loops;
    double thr_image_motion;     // [percent of the image width]
    bool log_clouds;
    KeypointTracker::Parameter kt_param;
    RigidTransformationRANSAC::Parameter rt_param;
    LoopClosingRT::Parameter lc_param;
    Parameter(double _min_total_score=10, int _min_tiles_used=3, 
      double _angle_init_keyframe=7.5, bool _detect_loops=true,
      double _thr_image_motion=0.25, bool _log_clouds=false,
      const KeypointTracker::Parameter &_kt_param=KeypointTracker::Parameter(),
      const RigidTransformationRANSAC::Parameter &_rt_param=RigidTransformationRANSAC::Parameter(0.005),
      const LoopClosingRT::Parameter &_lc_param=LoopClosingRT::Parameter())
    : min_total_score(_min_total_score), min_tiles_used(_min_tiles_used),
      angle_init_keyframe(_angle_init_keyframe), detect_loops(_detect_loops),
      thr_image_motion(_thr_image_motion), log_clouds(_log_clouds),
      kt_param(_kt_param), rt_param(_rt_param), lc_param(_lc_param) {}
  };

private:
  Parameter param;
  
  Scene::Ptr scene;

  cv::Mat_<cv::Vec3b> image;
  cv::Mat_<unsigned char> im_gray;

  std::vector<Eigen::Vector3f> src_pts;
  std::vector<Eigen::Vector3f> tgt_pts;

  double thr_image_motion_px;

  RigidTransformationRANSAC::Ptr rt;
  KeypointTracker::Ptr keytracker;
  LoopClosingRT::Ptr loopclosing;
  #ifndef KP_NO_CERES_AVAILABLE
  BundleAdjusterRT::Ptr bundler;
  #endif

  void lookupPoints3D(const DataMatrix2D<PointXYZRGB> &cloud, View &view);
  void setKeyframe(const DataMatrix2D<PointXYZRGB> &cloud, Scene &scene);
  void trackPose(View &keyframe, View &view);
  void getPoints(View &keyframe, View &view, 
        std::vector<Eigen::Vector3f> &src_pts, std::vector<Eigen::Vector3f> &tgt_pts);  
  void removeOutlierLinks(View &keyframe, View &view, double &score, 
        std::vector<double> &score_per_tile, double &dist_px);
  double getDeltaViewingAngle(Eigen::Matrix4f &pose1, Eigen::Matrix4f &pose2);
  bool isTrackOK(int num_tiles, const double &total_score, const std::vector<double> &score_per_tile);


public:
  cv::Mat dbg;

  CameraTrackerRGBD(const Parameter &p=Parameter());
  ~CameraTrackerRGBD();

  bool track(const DataMatrix2D<PointXYZRGB> &cloud, Eigen::Matrix4f &pose, 
        const cv::Mat_<unsigned char> &mask=cv::Mat());
  void doFullBundleAdjustment();
  /** isKeyframe returns true if the last tracked frame is a keyframe **/
  bool isKeyframe() {return scene->views.back()->is_keyframe;}

 
  Scene::Ptr &getSharedData() {return scene;}
  void setSharedData(Scene::Ptr &_scene); 

  typedef SmartPtr< ::kp::CameraTrackerRGBD> Ptr;
  typedef SmartPtr< ::kp::CameraTrackerRGBD const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

