/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CLUSTERING_MEAN_SHIFT_HH
#define KP_CLUSTERING_MEAN_SHIFT_HH

#include <iostream>
#include <vector>
#include <float.h>
#include "Clustering.hh"



namespace kp
{

class ClusteringMeanShift : public Clustering
{
public:
  class Parameter
  {   
  public:
    float lambda;           // flat part of the kernel
    float sigma;            // gaussian kernel
    float thr_prune;        // low value for pruning (e.g. 0.01*lamda)
    float init_p;           // percentage used for init
    int max_iter;           // max number of iterations
    double eps_converge;    // convergence threshold (e.g. 0.001*lamda)
    Parameter(float _lambda=0.1, float _sigma=0.03, float _thr_prune=0.003, 
      float _init_p=1., int _max_iter=500, float _eps_converge=0.0001) 
    : lambda(_lambda), sigma(_sigma), thr_prune(_thr_prune), 
      init_p(_init_p), max_iter(_max_iter), eps_converge(_eps_converge) {}
  };

private:
  float sqr_lambda;
  float inv_sqr_sigma;
  float sqr_thr_pruning;
  float init_pcent;
  int max_iter;
  float max_dist;

  std::vector< Cluster::Ptr > clusters;

  void initClusters(const Eigen::MatrixXf &data, std::vector< Cluster::Ptr > &clusters);
  int updateMean(const Eigen::MatrixXf &data, Eigen::VectorXf &center);

  inline float truncatedGaussian( const float &sqr_dist );


public:
  ClusteringMeanShift();
  ClusteringMeanShift(const Parameter &p = Parameter());
  ~ClusteringMeanShift();

  virtual void cluster(const DataMatrix2Df &samples);
  virtual void getClusters(std::vector<std::vector<int> > &_clusters);
  virtual void getCenters(DataMatrix2Df &_centers);
};




/*********************** INLINE METHODES **************************/

/*inline bool ClusteringMeanShift::IsConverge(double *d1, double *d2, unsigned dSize)
{
  for (unsigned i=0; i<dSize; i++)
    if (fabs(*d1-*d2) > maxDist)
      return false;

  return true;
}*/

inline float ClusteringMeanShift::truncatedGaussian( const float &sqr_dist)
{
  if (sqr_dist < sqr_lambda)
  {
    return exp(-inv_sqr_sigma*sqr_dist);
  }
  
  return 0.; 
}




}

#endif

