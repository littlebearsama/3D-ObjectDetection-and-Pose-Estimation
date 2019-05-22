/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_PLANE_ESTIMATION_RANSAC_HH
#define KP_PLANE_ESTIMATION_RANSAC_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "SmartPtr.hpp"




namespace kp
{

/**
 * PlaneEstimationRANSAC
 */
class PlaneEstimationRANSAC
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

  void computeCovarianceMatrix (const std::vector<Eigen::Vector3f> &pts, 
        const std::vector<int> &indices, const Eigen::Vector3f &mean, Eigen::Matrix3f &cov);
  void getInliers(std::vector<float> &dists, std::vector<int> &inliers);
  unsigned countInliers(std::vector<float> &dists);
  void getDistances(const std::vector<Eigen::Vector3f> &pts, 
        const Eigen::Vector3f &pt, const Eigen::Vector3f &n, std::vector<float> &dists);
  void getRandIdx(int size, int num, std::vector<int> &idx);
  void ransac(const std::vector<Eigen::Vector3f> &pts,
        Eigen::Vector3f &pt, Eigen::Vector3f &n, std::vector<int> &inliers);

  inline bool contains(const std::vector<int> &idx, int num);
  inline float sqr(const float &d) {return d*d;}


public:
  Parameter param;

  PlaneEstimationRANSAC(const Parameter &p=Parameter());
  ~PlaneEstimationRANSAC();

  void estimatePlaneLS(
        const std::vector<Eigen::Vector3f> &pts,
        const std::vector<int> &indices,
        Eigen::Vector3f &pt, Eigen::Vector3f &n);

  void ransacPlane(
        const std::vector<Eigen::Vector3f> &pts,
        Eigen::Vector3f &pt, Eigen::Vector3f &n, std::vector<int> &inliers);

  inline void explicitToImplicit(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, 
                const Eigen::Vector3f &pt3, float &a, float &b, float &c, float &d);
  inline void explicitToNormal(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, 
                const Eigen::Vector3f &pt3, Eigen::Vector3f &n); 
  inline float implicitPointDist(const float  &a, const float &b, 
                const float &c, const float &d, const Eigen::Vector3f &pt);
  inline float normalPointDist(const Eigen::Vector3f &pt, const Eigen::Vector3f &n, 
                const Eigen::Vector3f &pt_dist);

  typedef SmartPtr< ::kp::PlaneEstimationRANSAC> Ptr;
  typedef SmartPtr< ::kp::PlaneEstimationRANSAC const> ConstPtr;

};




/*********************** INLINE METHODES **************************/
inline bool PlaneEstimationRANSAC::contains(const std::vector<int> &idx, int num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}

/**
 * explicitToImplicit
 */
inline void PlaneEstimationRANSAC::explicitToImplicit(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, const Eigen::Vector3f &pt3, float &a, float &b, float &c, float &d)
{
  a = ( pt2[1] - pt1[1] ) * ( pt3[2] - pt1[2] )
    - ( pt2[2] - pt1[2] ) * ( pt3[1] - pt1[1] );

  b = ( pt2[2] - pt1[2] ) * ( pt3[0] - pt1[0] )
    - ( pt2[0] - pt1[0] ) * ( pt3[2] - pt1[2] );

  c = ( pt2[0] - pt1[0] ) * ( pt3[1] - pt1[1] )
    - ( pt2[1] - pt1[1] ) * ( pt3[0] - pt1[0] );

  d = - pt2[0] * a - pt2[1] * b - pt2[2] * c;
}

/**
 * explicitToNormal
 */
inline void PlaneEstimationRANSAC::explicitToNormal(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, const Eigen::Vector3f &pt3, Eigen::Vector3f &n)
{
  float norm;

  n[0] = ( pt2[1] - pt1[1] ) * ( pt3[2] - pt1[2] )
       - ( pt2[2] - pt1[2] ) * ( pt3[1] - pt1[1] );

  n[1] = ( pt2[2] - pt1[2] ) * ( pt3[0] - pt1[0] )
       - ( pt2[0] - pt1[0] ) * ( pt3[2] - pt1[2] );

  n[2] = ( pt2[0] - pt1[0] ) * ( pt3[1] - pt1[1] )
       - ( pt2[1] - pt1[1] ) * ( pt3[0] - pt1[0] );

  norm = sqrt ( sqr(n[0]) + sqr(n[1]) + sqr(n[2] ) );

  if ( fabs(norm) <= std::numeric_limits<float>::epsilon() )
  {
    n[0] = n[1] = n[2] = std::numeric_limits<float>::quiet_NaN();;
  }
  else
  {
    norm = 1./norm;
    n[0] *= norm;
    n[1] *= norm;
    n[2] *= norm;
  }
}



/**
 * implicitPointDist
 */
inline float PlaneEstimationRANSAC::implicitPointDist(const float  &a, const float &b, const float &c, const float &d, const Eigen::Vector3f &pt)
{
  return fabs ( a * pt[0] + b * pt[1] + c * pt[2] + d ) /
         sqrt ( a * a + b * b + c * c );
}

/**
 * normalPointDist
 */
inline float PlaneEstimationRANSAC::normalPointDist(const Eigen::Vector3f &pt, const Eigen::Vector3f &n, const Eigen::Vector3f &pt_dist)
{
  return (pt_dist-pt).dot(n);
}


}

#endif

