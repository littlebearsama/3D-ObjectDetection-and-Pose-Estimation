/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_STRUCTURE_FROM_MOTION_INIT_AR_HH
#define KP_STRUCTURE_FROM_MOTION_INIT_AR_HH

#include <stdexcept>
#include <string>
#include "Scene.hh"
#include "View.hh"
#include "KeypointTracker.hh"
#include "CameraTrackerPnP.hh"
#include "ARMarkerDetector.hh"
#include "Triangulation.hh"
#include "ProjBundleAdjuster.hh"


namespace kp
{

class StructureFromMotionInitAR 
{
public:
  class Parameter
  {
  public:
    int min_points_keyframe;     // a keyfame must have a minimum of e.g. 100 3d points
    float delta_angle_keyframe;  // min. angle between two last and current keyframe [°]
    std::string ar_pattern;      // config file for the ar pattern
    bool full_ba_optimize_cam;
    bool clean_up_data;          // clean up shared data and only keep keyframes
    bool clean_up_images;        // clean up the images and only keep keyframes
    KeypointTracker::Parameter kt_param;
    CameraTrackerPnP::Parameter ct_param;
    ARMarkerDetector::Parameter ar_param;
    Triangulation::Parameter tr_param;
    ProjBundleAdjuster::Parameter ba_param;
    Parameter(int _min_points_keyframe=100, float _delta_angle_keyframe=7.5,
      const std::string &_ar_pattern="../v4r/armarker/pattern/Pattern8.cfg",
      bool _full_ba_optimize_cam=false,
      bool _clean_up_data=true, bool _clean_up_images=true,
      const KeypointTracker::Parameter &_kt_param=KeypointTracker::Parameter(),
      const CameraTrackerPnP::Parameter &_ct_param=CameraTrackerPnP::Parameter(),
      const ARMarkerDetector::Parameter &_ar_param=ARMarkerDetector::Parameter(),
      const Triangulation::Parameter &_tr_param=Triangulation::Parameter(),
      const ProjBundleAdjuster::Parameter &_ba_param=ProjBundleAdjuster::Parameter() )
    : min_points_keyframe(_min_points_keyframe), delta_angle_keyframe(_delta_angle_keyframe),
      ar_pattern(_ar_pattern),full_ba_optimize_cam(_full_ba_optimize_cam), 
      clean_up_data(_clean_up_data), clean_up_images(_clean_up_images),
      kt_param(_kt_param), ct_param(_ct_param), ar_param(_ar_param), tr_param(_tr_param),
      ba_param(_ba_param) {}
  };

private:
  Parameter param;
  
  cv::Mat_<cv::Vec3b> image;
  cv::Mat_<unsigned char> im_gray;
  cv::Mat im_dbg;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;

  Scene::Ptr scene;

  KeypointTracker::Ptr keytracker;
  CameraTrackerPnP::Ptr camtracker;
  #ifndef KP_NO_ARMARKER
  ARMarkerDetector::Ptr ardetector;
  #endif
  Triangulation::Ptr tri;
  #ifndef KP_NO_CERES_AVAILABLE
  ProjBundleAdjuster::Ptr bundler;
  ProjBundleAdjuster::Ptr full_bundler;
  #endif

  void setKeyframe(const cv::Mat_<unsigned char> &im, Scene &scene);
  double getDeltaViewingAngle(Scene &scene, View &keyframe,
        Eigen::Matrix4f &pose1, Eigen::Matrix4f &pose2);
  void cleanUpDate(Scene &scene);
  void cleanUpImages(Scene &scene);

public:

  StructureFromMotionInitAR(const Parameter &p=Parameter());
  ~StructureFromMotionInitAR();

  bool track(const cv::Mat &_image, Eigen::Matrix4f &pose, const cv::Mat &mask=cv::Mat());
  void doFullBundleAdjustment();

  Scene::Ptr &getSharedData() {return scene;}
  void setSharedData(Scene::Ptr &_scene);

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  void setDebugImage(const cv::Mat _image);

  typedef SmartPtr< ::kp::StructureFromMotionInitAR> Ptr;
  typedef SmartPtr< ::kp::StructureFromMotionInitAR const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

