/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_SEARCH_KDTREE_FLANN_2f_HH
#define KP_SEARCH_KDTREE_FLANN_2f_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include "SmartPtr.hpp"




namespace kp
{

/**
 * SearchKdTreeFLANN2f
 */
class SearchKdTreeFLANN2f
{
public:
  class Parameter
  {
  public:
    bool sorted;
    float epsilon;
    Parameter(bool _sorted=true, float _epsilon=0.)
    : sorted(_sorted), epsilon(_epsilon) {}
  };

  typedef ::flann::Index< ::flann::L2_Simple<float> > FLANNIndex;
  typedef SmartPtr< ::kp::SearchKdTreeFLANN2f > Ptr;
  typedef SmartPtr< ::kp::SearchKdTreeFLANN2f const> ConstPtr;

private:
  Parameter param;

  FLANNIndex* flann_index;
  float* data;

  std::vector<int> index_mapping;
  bool identity_mapping;

  int dim;
  int total_nr_points;

  ::flann::SearchParams param_k;
  ::flann::SearchParams param_radius;

  void convertPointsToArray(const std::vector<Eigen::Vector2f> &points);


public:

  void clean();
  void setInputPoints(const std::vector<Eigen::Vector2f> &points);

  void nearestKSearch(const Eigen::Vector2f &pt, int k, std::vector<int> &k_indices, 
        std::vector<float> &k_distances);
  void nearestKSearch(int idx, int k, std::vector<int> &k_indices, std::vector<float> &k_distances);

  void radiusSearch(const Eigen::Vector2f &pt, double radius, 
        std::vector<int> &k_indices, std::vector<float> &k_sqr_dists, unsigned int max_nn=0);
  void radiusSearch(int idx, double radius,
        std::vector<int> &k_indices, std::vector<float> &k_sqr_dists, unsigned int max_nn=0);



  SearchKdTreeFLANN2f(const Parameter &p=Parameter());
  ~SearchKdTreeFLANN2f();
};




/*********************** INLINE METHODES **************************/



}

#endif

