/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_REFINE_PROJECTED_POINT_LOCATION_LK_BASE_HH
#define KP_REFINE_PROJECTED_POINT_LOCATION_LK_BASE_HH

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "v4r/KeypointTools/SmartPtr.hpp"



namespace kp
{

/**
 * RefineProjectedPointLocationLKbase
 */
class RefineProjectedPointLocationLKbase
{
public:

public:
  RefineProjectedPointLocationLKbase(){};
  virtual ~RefineProjectedPointLocationLKbase(){};

  virtual void setSourceImage(const cv::Mat_<unsigned char> &_im_src, const Eigen::Matrix4f &_pose_src) {};
  virtual void setTargetImage(const cv::Mat_<unsigned char> &_im_tgt, const Eigen::Matrix4f &_pose_tgt) {};
  virtual void refineImagePoints(const std::vector<Eigen::Vector3f> &pts, 
        const std::vector<Eigen::Vector3f> &normals, 
        std::vector<cv::Point2f> &im_pts_tgt, std::vector<int> &converged) {};

  virtual void setSourceCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs) {};
  virtual void setTargetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs) {};

  typedef SmartPtr< ::kp::RefineProjectedPointLocationLKbase> Ptr;
  typedef SmartPtr< ::kp::RefineProjectedPointLocationLKbase const> ConstPtr;
};




/*********************** INLINE METHODES **************************/


}

#endif

