/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_RIGID_TRANSFORMATION_RANSAC_HH
#define KP_RIGID_TRANSFORMATION_RANSAC_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "SmartPtr.hpp" 


namespace kp
{

/**
 * RigidTransformationRANSAC
 */
class RigidTransformationRANSAC
{
public:
  class Parameter
  {
  public:
    double inl_dist;
    double eta_ransac;               // eta for pose ransac
    unsigned max_rand_trials;         // max. number of trials for pose ransac

    Parameter(double _inl_dist=0.01, double _eta_ransac=0.01, unsigned _max_rand_trials=10000)
     : inl_dist(_inl_dist), eta_ransac(_eta_ransac), max_rand_trials(_max_rand_trials) {}
  };

private:
  Eigen::Matrix4f invPose;

  void DemeanPoints (
        const std::vector<Eigen::Vector3f > &inPts, 
        const std::vector<int> &indices,
        const Eigen::Vector3f &centroid,
        Eigen::MatrixXf &outPts);

  void ComputeCentroid(
        const std::vector<Eigen::Vector3f > &pts, 
        const std::vector<int> &indices,
        Eigen::Vector3f &centroid);

  void Ransac(
        const std::vector<Eigen::Vector3f > &srcPts,
        const std::vector<Eigen::Vector3f > &tgtPts,
        Eigen::Matrix4f &transform,
        std::vector<int> &inliers);

  void GetDistances(
        const std::vector<Eigen::Vector3f > &srcPts,
        const std::vector<Eigen::Vector3f > &tgtPts,
        const Eigen::Matrix4f &transform,
        std::vector<float> &dists);

  void GetInliers(std::vector<float> &dists, std::vector<int> &inliers);
  unsigned CountInliers(std::vector<float> &dists);
  void GetRandIdx(int size, int num, std::vector<int> &idx);


  inline bool Contains(const std::vector<int> &idx, int num);
  inline void InvPose(const Eigen::Matrix4f &pose, Eigen::Matrix4f &invPose);



public:
  Parameter param;

  RigidTransformationRANSAC(Parameter p=Parameter());
  ~RigidTransformationRANSAC();

  void estimateRigidTransformationSVD(
        const std::vector<Eigen::Vector3f > &srcPts,
        const std::vector<int> &srcIndices,
        const std::vector<Eigen::Vector3f > &tgtPts,
        const std::vector<int> &tgtIndices,
        Eigen::Matrix4f &transform);

  void compute(
        const std::vector<Eigen::Vector3f > &srcPts,
        const std::vector<Eigen::Vector3f > &tgtPts,
        Eigen::Matrix4f &transform,
        std::vector<int> &inliers);

  typedef SmartPtr< ::kp::RigidTransformationRANSAC> Ptr;
  typedef SmartPtr< ::kp::RigidTransformationRANSAC const> ConstPtr;
};




/*********************** INLINE METHODES **************************/
inline bool RigidTransformationRANSAC::Contains(const std::vector<int> &idx, int num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}

inline void RigidTransformationRANSAC::InvPose(const Eigen::Matrix4f &pose, Eigen::Matrix4f &invPose)
{ 
  invPose.setIdentity();
  invPose.block<3, 3> (0, 0) = pose.block<3, 3> (0, 0).transpose();
  invPose.block<3, 1> (0, 3) = -1*(invPose.block<3, 3> (0, 0)*pose.block<3, 1> (0, 3));
}


}

#endif

