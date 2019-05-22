/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_PROJ_BUNDLE_ADJUSTER_HH
#define KP_PROJ_BUNDLE_ADJUSTER_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#ifndef KP_NO_CERES_AVAILABLE
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif

#include "v4r/KeypointBase/LinkedKeypoint.hh"
#include "Scene.hh"
#include "View.hh"

namespace kp
{

class ProjBundleAdjuster 
{
public:
  class Parameter
  {
  public:
    int inc_bundle_frames;     // INT_MAX -> bundle all keyframe-cameras
    bool optimize_camera;
    bool optimize_struct_only;
    Parameter(int _inc_bundle_frames=10, bool _optimize_camera=false, 
       bool _optimize_struct_only=false) 
     : inc_bundle_frames(_inc_bundle_frames), optimize_camera(_optimize_camera),
       optimize_struct_only(_optimize_struct_only) {}
  };
  class Camera
  {
  public:
    int idx;
    Eigen::Matrix<double, 6, 1> pose_Rt;
  };

private:
  Parameter param;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;  

  std::vector<double> camera_intrinsics;

  Scene::Ptr scene;

  std::vector<Camera> cameras;

  void getDataToBundle(const Scene &scene, std::vector<Camera> &cameras);
  void setBundledData(const std::vector<Camera> &cameras, Scene &scene);
  void bundle(Scene &scene, std::vector<Camera> &cameras);


  /*void getDataToBundle(const std::vector<View::Ptr> &views, std::vector<Camera> &cams, 
        std::vector<Point3D> &pts, std::vector<ImagePoint> &im_pts, std::vector<View::Ptr> &kfs); 
  void copyBackBundledData(const std::vector<Camera> &cams, 
        const std::vector<Point3D> &pts, std::vector<View::Ptr> &kfs);
  void bundle(const std::vector<ImagePoint> &all_image_points, double *camera_intrinsics,
        std::vector<Camera> *all_cameras, std::vector<Point3D> *all_points);
  void packCamerasRotationAndTranslation(const std::vector<Camera> &all_cameras, 
        std::vector<Eigen::Matrix<double, 6, 1> > &all_cameras_R_t);
  void unpackCamerasRotationAndTranslation(
        const std::vector<Eigen::Matrix<double, 6, 1> > &all_cameras_R_t, 
        std::vector<Camera> &all_cameras);*/

  inline void getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R);
  inline void getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t);
  inline void setPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix4f &pose);


public:
  cv::Mat dbg;

  /*std::vector<Camera> all_cameras;
  std::vector<Point3D> all_points;
  std::vector<ImagePoint> all_image_points;*/

  ProjBundleAdjuster(const Parameter &p=Parameter());
  ~ProjBundleAdjuster();

  void optimize();

  Scene::Ptr &getSharedData() {return scene;}
  void setSharedData(Scene::Ptr &_scene) {scene = _scene;}
  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef SmartPtr< ::kp::ProjBundleAdjuster> Ptr;
  typedef SmartPtr< ::kp::ProjBundleAdjuster const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

inline void ProjBundleAdjuster::getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R)
{
  R(0,0) = pose(0,0); R(0,1) = pose(0,1); R(0,2) = pose(0,2);
  R(1,0) = pose(1,0); R(1,1) = pose(1,1); R(1,2) = pose(1,2);
  R(2,0) = pose(2,0); R(2,1) = pose(2,1); R(2,2) = pose(2,2);
}

inline void ProjBundleAdjuster::getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t)
{
  t(0) = pose(0,3);
  t(1) = pose(1,3);
  t(2) = pose(2,3);
}

inline void ProjBundleAdjuster::setPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix4f &pose)
{
  pose.setIdentity();
  pose(0,0) = R(0,0); pose(0,1) = R(0,1); pose(0,2) = R(0,2); pose(0,3) = t(0);
  pose(1,0) = R(1,0); pose(1,1) = R(1,1); pose(1,2) = R(1,2); pose(1,3) = t(1);
  pose(2,0) = R(2,0); pose(2,1) = R(2,1); pose(2,2) = R(2,2); pose(2,3) = t(2);
}

} //--END--

#endif

