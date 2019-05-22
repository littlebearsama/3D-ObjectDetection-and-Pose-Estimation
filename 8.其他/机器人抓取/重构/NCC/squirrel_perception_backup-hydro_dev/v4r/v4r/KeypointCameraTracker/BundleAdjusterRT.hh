/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_BUNDLE_ADJUSTER_RT_HH
#define KP_BUNDLE_ADJUSTER_RT_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#ifndef KP_NO_CERES_AVAILABLE
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif

#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointBase/LinkedKeypoint.hh"
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "View.hh"
#include "Scene.hh"

namespace kp
{

class BundleAdjusterRT 
{
public:
  class Parameter
  {
  public:
    Parameter() {}
  };

private:
  Parameter param;

  std::vector<View::Ptr> kfs;
  std::vector<unsigned> lt_camera;

  Scene::Ptr scene;

  void bundle(std::vector<View::Ptr> &kfs, std::vector<Eigen::Matrix<double, 6, 1> > &cameras, std::vector<unsigned> &lt);
  void getDataToBundle(const std::vector<View::Ptr> &views, std::vector<View::Ptr> &kfs,
        std::vector<Eigen::Matrix<double, 6, 1> > &cameras, std::vector<unsigned> &lt); 
  void copyBackBundledData(const std::vector<Eigen::Matrix<double, 6, 1> > &cameras, 
        std::vector<View::Ptr> &kfs);

  inline void getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R);
  inline void getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t);
  inline void setPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix4f &pose);


public:
  cv::Mat dbg;

  std::vector<Eigen::Matrix<double, 6, 1> > cameras;

  BundleAdjusterRT(const Parameter &p=Parameter());
  ~BundleAdjusterRT();

  void optimize();

  void setSharedData(const Scene::Ptr &_scene);

  typedef SmartPtr< ::kp::BundleAdjusterRT> Ptr;
  typedef SmartPtr< ::kp::BundleAdjusterRT const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

inline void BundleAdjusterRT::getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R)
{
  R(0,0) = pose(0,0); R(0,1) = pose(0,1); R(0,2) = pose(0,2);
  R(1,0) = pose(1,0); R(1,1) = pose(1,1); R(1,2) = pose(1,2);
  R(2,0) = pose(2,0); R(2,1) = pose(2,1); R(2,2) = pose(2,2);
}

inline void BundleAdjusterRT::getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t)
{
  t(0) = pose(0,3);
  t(1) = pose(1,3);
  t(2) = pose(2,3);
}

inline void BundleAdjusterRT::setPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix4f &pose)
{
  pose.setIdentity();
  pose(0,0) = R(0,0); pose(0,1) = R(0,1); pose(0,2) = R(0,2); pose(0,3) = t(0);
  pose(1,0) = R(1,0); pose(1,1) = R(1,1); pose(1,2) = R(1,2); pose(1,3) = t(1);
  pose(2,0) = R(2,0); pose(2,1) = R(2,1); pose(2,2) = R(2,2); pose(2,3) = t(2);
}

} //--END--

#endif

