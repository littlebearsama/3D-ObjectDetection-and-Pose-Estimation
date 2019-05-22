/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CLUSTERING_HH
#define KP_CLUSTERING_HH

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "DataMatrix2D.hpp"
#include "SmartPtr.hpp"


namespace kp 
{

/**
 * Cluster
 */
class Cluster
{
public:
  float sqr_sigma;
  Eigen::VectorXf data;
  std::vector<int> indices;

  Cluster() : sqr_sigma(0) {};
  Cluster(const Eigen::VectorXf &d) : sqr_sigma(0), data(d) {}
  Cluster(const Eigen::VectorXf &d, int idx) : sqr_sigma(0), data(d) {
    indices.push_back(idx);
  }

  typedef SmartPtr< ::kp::Cluster> Ptr;
  typedef SmartPtr< ::kp::Cluster const> ConstPtr;
};

/**
 * Clustering
 */
class Clustering
{
protected:
  std::vector< Cluster::Ptr > clusters;

public:
  Clustering() {}
  ~Clustering() {}

  virtual void cluster(const DataMatrix2Df &samples) {};
  virtual void getClusters(std::vector<std::vector<int> > &_clusters) {};
  virtual void getCenters(DataMatrix2Df &_centers) {};
};



}

#endif

