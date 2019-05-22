/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CIRCLE_3D_RANSAC_HH
#define KP_CIRCLE_3D_RANSAC_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "SmartPtr.hpp"




namespace kp
{

/**
 * Circle3D
 */
class Circle3D
{
public:
  class Parameter
  {
  public:
    double inl_dist;
    double eta_ransac;               // eta for pose ransac
    unsigned max_rand_trials;         // max. number of trials for pose ransac

    Parameter(double _inl_dist=0.001, double _eta_ransac=0.01, unsigned _max_rand_trials=10000)
     : inl_dist(_inl_dist), eta_ransac(_eta_ransac), max_rand_trials(_max_rand_trials) {}
  };

private:

  void getInliers(std::vector<float> &dists, std::vector<int> &inliers);
  unsigned countInliers(std::vector<float> &dists);
  void getDistances(const std::vector<Eigen::Vector3f> &pts, Eigen::Vector3f &center, Eigen::Vector3f &normal, float &radius, std::vector<float> &dists);


public:
  Parameter param;

  Circle3D(const Parameter &p=Parameter());
  ~Circle3D();

  int computeCircle(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, const Eigen::Vector3f &pt3, Eigen::Vector3f &center, Eigen::Vector3f &normal, float &radius);
  void ransacCircle(const std::vector<Eigen::Vector3f> &pts, Eigen::Vector3f &center, Eigen::Vector3f &normal, float &radius, std::vector<int> &inliers);

  typedef SmartPtr< ::kp::Circle3D> Ptr;
  typedef SmartPtr< ::kp::Circle3D const> ConstPtr;

};




/*********************** INLINE METHODES **************************/

}

#endif

