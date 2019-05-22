/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_INV_POSE_HPP
#define KP_INV_POSE_HPP

#include <Eigen/Dense>
#include "v4r/KeypointTools/rotation.h"

namespace kp
{

inline void invPose(const Eigen::Matrix4f &pose, Eigen::Matrix4f &inv_pose)
{
  inv_pose.setIdentity();
  inv_pose.topLeftCorner<3,3>() = pose.topLeftCorner<3,3>().transpose();
  inv_pose.block<3, 1> (0, 3) = -1*( inv_pose.topLeftCorner<3,3>()*pose.block<3, 1>(0,3) );
}

inline void invPose(const Eigen::Matrix4d &pose, Eigen::Matrix4d &inv_pose)
{
  inv_pose.setIdentity();
  inv_pose.topLeftCorner<3,3>() = pose.topLeftCorner<3,3>().transpose();
  inv_pose.block<3, 1> (0, 3) = -1*( inv_pose.topLeftCorner<3,3>()*pose.block<3, 1>(0,3) );
}

/**
 * invPose6
 */
template<typename T1,typename T2, typename T3, typename T4>
inline void invPose6(const T1 r[3], const T2 t[3], T3 inv_r[3], T4 inv_t[3])
{
  inv_r[0] = T1(-1)*r[0];
  inv_r[1] = T1(-1)*r[1];
  inv_r[2] = T1(-1)*r[2];

  kp::AngleAxisRotatePoint(inv_r, t, inv_t);
  
  inv_t[0] = T4(-1)*inv_t[0];
  inv_t[1] = T4(-1)*inv_t[1];
  inv_t[2] = T4(-1)*inv_t[2];
}


} //--END--

#endif




