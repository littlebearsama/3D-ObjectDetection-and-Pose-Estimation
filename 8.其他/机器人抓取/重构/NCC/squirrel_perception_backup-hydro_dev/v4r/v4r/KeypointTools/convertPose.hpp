/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CONVERT_POSE_HPP
#define KP_CONVERT_POSE_HPP

#include <Eigen/Dense>
#include "v4r/KeypointTools/rotation.h"
#include <opencv2/core/core.hpp>

namespace kp
{

inline void convertPose(const Eigen::Matrix4f &pose, Eigen::Matrix<double, 6, 1> &rt)
{
  Eigen::Matrix3d R = pose.topLeftCorner<3,3>().cast<double>();
  RotationMatrixToAngleAxis(&R(0,0), &rt[0]);
  rt.tail<3>() = pose.block<3,1>(0,3).cast<double>();
}

inline void convertPose(const Eigen::Matrix4f &pose, Eigen::VectorXd &rt)
{
  rt.resize(6); 
  Eigen::Matrix3d R = pose.topLeftCorner<3,3>().cast<double>();
  RotationMatrixToAngleAxis(&R(0,0), &rt[0]);
  rt.tail<3>() = pose.block<3,1>(0,3).cast<double>();
}

inline void convertPose(const Eigen::Matrix<double, 6, 1> &rt, Eigen::Matrix4f &pose)
{
  Eigen::Matrix3d R;
  pose.setIdentity();
  AngleAxisToRotationMatrix(&rt[0], &R(0,0));
  pose.topLeftCorner<3,3>() = R.cast<float>();
  pose.block<3,1>(0,3) = rt.tail<3>().cast<float>();
}

/**
 * convertPose
 */
inline void convertPose(const cv::Mat_<double> &R, const cv::Mat_<double> &t, Eigen::Matrix4f &pose)
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




