/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_IMAGE_TRANSFORM_RANSAC_HH
#define KP_IMAGE_TRANSFORM_RANSAC_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "SmartPtr.hpp"


namespace kp
{

/**
 * ImageTransformRANSAC
 */
class ImageTransformRANSAC
{
public:
  class Parameter
  {
  public:
    double inl_dist;
    double eta_ransac;               // eta for pose ransac
    unsigned max_rand_trials;         // max. number of trials for pose ransac

    Parameter(double _inl_dist=3, double _eta_ransac=0.01, unsigned _max_rand_trials=10000)
      : inl_dist(_inl_dist), eta_ransac(_eta_ransac), max_rand_trials(_max_rand_trials) {}
  };

private:
  static double SQRT2;

  void getDistances(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        const Eigen::Matrix3f &transform,
        std::vector<float> &dists);

  void getInliers(std::vector<float> &dists, std::vector<int> &inliers);
  unsigned countInliers(std::vector<float> &dists);
  void getRandIdx(int size, int num, std::vector<int> &idx);

  void normalizePoints(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > &pts_in,
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > &pts_out,
        Eigen::Matrix3d &T);

  inline bool contains(const std::vector<int> &idx, int num);



public:
  Parameter param;

  ImageTransformRANSAC(Parameter p=Parameter());
  ~ImageTransformRANSAC();

  void estimateSimilarityLS(
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
          const std::vector<int> &src_indices,
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
          const std::vector<int> &tgt_indices,
          Eigen::Matrix3f &transform);
  void estimateSimilarityLS(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        Eigen::Matrix3f &transform);

  void estimateAffineLS(
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
          const std::vector<int> &src_indices,
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
          const std::vector<int> &tgt_indices,
          Eigen::Matrix3f &transform);
  void estimateAffineLS(
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
          Eigen::Matrix3f &transform);

  void estimateHomographyLS(
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
          const std::vector<int> &src_indices,
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
          const std::vector<int> &tgt_indices,
          Eigen::Matrix3f &transform);

  void ransacSimilarity(
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
          Eigen::Matrix3f &transform, std::vector<int> &inliers);
  void ransacAffine(
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
          Eigen::Matrix3f &transform, std::vector<int> &inliers);
  void ransacHomography(
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
          const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
          Eigen::Matrix3f &transform, std::vector<int> &inliers);

  typedef SmartPtr< ::kp::ImageTransformRANSAC> Ptr;
  typedef SmartPtr< ::kp::ImageTransformRANSAC const> ConstPtr;
};




/*********************** INLINE METHODES **************************/
inline bool ImageTransformRANSAC::contains(const std::vector<int> &idx, int num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}



}

#endif

