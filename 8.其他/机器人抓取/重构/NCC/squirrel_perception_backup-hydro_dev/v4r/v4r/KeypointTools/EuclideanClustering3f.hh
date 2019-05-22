/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_EUCLIDEAN_CLUSTERING3f_HH
#define KP_EUCLIDEAN_CLUSTERING3f_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "SmartPtr.hpp"
#include "SearchKdTreeFLANN3f.hh"




namespace kp
{

/**
 * EuclideanClustering3f
 */
class EuclideanClustering3f
{
public:
  class Parameter
  {
  public:
    float max_distance;
    unsigned min_points_per_cluster;
    unsigned max_points_per_cluster;
    SearchKdTreeFLANN3f::Parameter flannParam;
    Parameter(float _max_distance=.005, unsigned _min_points_per_cluster=10, 
      unsigned _max_points_per_cluster=UINT_MAX,
      const SearchKdTreeFLANN3f::Parameter &_flannParam=SearchKdTreeFLANN3f::Parameter())
    : max_distance(_max_distance), min_points_per_cluster(_min_points_per_cluster), 
      max_points_per_cluster(_max_points_per_cluster),
      flannParam(_flannParam) {}
  };

private:

  SearchKdTreeFLANN3f::Ptr tree;
  unsigned nb_points;


public:
  Parameter param;

  EuclideanClustering3f(const Parameter &p=Parameter());
  ~EuclideanClustering3f();

  void setInputPoints(const std::vector<Eigen::Vector3f> &points);
  void extract(std::vector< std::vector<int> > &clusters);

  typedef SmartPtr< ::kp::EuclideanClustering3f> Ptr;
  typedef SmartPtr< ::kp::EuclideanClustering3f const> ConstPtr;
};




/*********************** INLINE METHODES **************************/


}

#endif

